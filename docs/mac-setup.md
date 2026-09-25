# AERIS — macOS development environment setup

Tested and recorded during Phase 1 on the machine described in
`AERIS_TECHNICAL_SPEC.md` Appendix A (MacBook Air, Apple M5, 16 GB,
macOS 27.2, Homebrew 7.0.6). Re-verify on a different machine.

This is the **exact, working sequence** — it deviates from PX4's own
published docs in ways explained inline, because the tagged release's
setup script and current Homebrew have drifted apart. See
`third_party/patches/px4/README.md` for the source-level compatibility
patches this depends on.

## 1. Prerequisites

```bash
xcode-select --install       # if not already installed
brew --version                # confirm Homebrew is present
```

Raise the open-file limit (PX4's build opens many files at once):

```bash
echo '' >> ~/.zshrc
echo '# AERIS / PX4: raise open-file limit for SITL builds' >> ~/.zshrc
echo 'ulimit -S -n 2048' >> ~/.zshrc
```

(On this machine `ulimit -n` was already 1,048,576 via prior shell config, so
this was a no-op safety net, not a fix for an active problem — check yours
with `ulimit -n` first.)

## 2. Clone PX4

```bash
mkdir -p ~/aeris-deps && cd ~/aeris-deps
git clone --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot
cd PX4-Autopilot
git submodule update --init --recursive --force   # ~2.7 GB total
```

## 3. Replace the stale v1.17.0 setup script with `main`'s

**Why:** the v1.17.0 tag's `Tools/setup/macos.sh` calls the now-deprecated,
no-op `px4-dev`/`px4-sim` Homebrew meta-formulae and doesn't handle
Homebrew 6.0+'s "untrusted tap" gate. It silently installs almost nothing.
`main`'s version (rewritten specifically for this) works.

```bash
cd ~/aeris-deps/PX4-Autopilot
git remote set-branches --add origin main
git fetch origin main --depth 1
mkdir -p Tools/setup/_v1.17.0_original
for f in macos.sh requirements.txt; do
  cp Tools/setup/$f Tools/setup/_v1.17.0_original/
done
for f in macos.sh gz-tap-pin.txt protobuf-pin.txt requirements.txt; do
  git show origin/main:Tools/setup/$f > Tools/setup/$f
done
chmod +x Tools/setup/macos.sh
```

## 4. Apply the PX4 source patches

```bash
for p in <AERIS_REPO>/third_party/patches/px4/*.patch; do
  git apply "$p"
done
```

See `third_party/patches/px4/README.md` for exactly what each one fixes and
why. All 9 are build-system/toolchain fixes; none change flight-control,
estimation, or simulation behavior.

## 5. Run the (replaced) setup script

```bash
./Tools/setup/macos.sh --sim-tools
```

This installs, via Homebrew: the PX4 toolchain (cmake, ninja, arm-gcc,
fastdds, genromfs, kconfig-frontends, ...), Gazebo Harmonic (pinned tap
commit, see `configs/versions.lock.yaml`), and (if it can) XQuartz.

**Known gaps in this step on this machine:**

- **XQuartz's cask install needs an interactive `sudo` password** the script
  cannot supply non-interactively. If you see `sudo: a password is required`,
  run it yourself:
  ```bash
  brew install --cask xquartz
  ```
  This is **only needed for the Gazebo GUI** — headless server mode (the
  path AERIS uses for all automated work) does not need it.
- `opencv@4` may show `Error: The brew link step did not complete
  successfully` because of a pre-existing `cv2` install from an unrelated
  project in this machine's system Python site-packages. This is harmless —
  `opencv@4` is keg-only by design and PX4's build finds it via
  `CMAKE_PREFIX_PATH` (step 6), not the top-level symlink.

## 6. Fix protobuf's pinned version, and set the keg-only env block

The script's `protobuf-pin.txt` step needs a local `homebrew-core` git clone
to work automatically, which this Homebrew installation doesn't have
(API-only mode). Pin it manually via a throwaway local tap:

```bash
curl -fsSL "https://raw.githubusercontent.com/Homebrew/homebrew-core/88d598cf1d2dba7063fbc29d441d072963481123/Formula/p/protobuf.rb" \
  -o /tmp/protobuf_pin.rb
brew tap-new local/protobufpin
cp /tmp/protobuf_pin.rb "$(brew --repo local/protobufpin)/Formula/protobuf.rb"
brew reinstall local/protobufpin/protobuf   # installs 36.1
```

Then get `libopencv_dnn`'s protobuf 36.2 dependency working *alongside*
36.1 (needed only by `libOpticalFlowSystem.dylib`; see
`third_party/patches/px4/README.md` for the full rationale):

```bash
brew fetch protobuf   # downloads the current (36.2) bottle without installing it
BOTTLE=$(brew --cache protobuf 2>/dev/null || \
  find ~/Library/Caches/Homebrew/downloads -iname "*protobuf*36.2*bottle.tar.gz" | head -1)
mkdir -p /tmp/protobuf362_extract && cd /tmp/protobuf362_extract
tar xzf "$BOTTLE"
cd /opt/homebrew/Cellar/protobuf/36.1/lib
cp /tmp/protobuf362_extract/protobuf/36.2/lib/libprotobuf.36.2.0.dylib .
cp /tmp/protobuf362_extract/protobuf/36.2/lib/libutf8_validity.36.2.0.dylib .
chmod +w libprotobuf.36.2.0.dylib libutf8_validity.36.2.0.dylib
install_name_tool -id "/opt/homebrew/opt/protobuf/lib/libprotobuf.36.2.0.dylib" libprotobuf.36.2.0.dylib
otool -L libprotobuf.36.2.0.dylib | grep "@@HOMEBREW_PREFIX@@" | awk '{print $1}' | while read old; do
  install_name_tool -change "$old" "${old/@@HOMEBREW_PREFIX@@//opt/homebrew}" libprotobuf.36.2.0.dylib
done
install_name_tool -id "/opt/homebrew/opt/protobuf/lib/libutf8_validity.36.2.0.dylib" libutf8_validity.36.2.0.dylib
codesign --force --sign - libprotobuf.36.2.0.dylib libutf8_validity.36.2.0.dylib
chmod -w libprotobuf.36.2.0.dylib libutf8_validity.36.2.0.dylib
```

Then, in every shell you build/run PX4 SITL from (add to your build script
or a project `.envrc`), because `opencv@4` and `qt@5` are keg-only:

```bash
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/opencv@4:/opt/homebrew/opt/qt@5:${CMAKE_PREFIX_PATH}"
export LDFLAGS="-L/opt/homebrew/opt/opencv@4/lib -L/opt/homebrew/opt/qt@5/lib"
export CPPFLAGS="-I/opt/homebrew/opt/opencv@4/include -I/opt/homebrew/opt/qt@5/include"
export PKG_CONFIG_PATH="/opt/homebrew/opt/opencv@4/lib/pkgconfig:/opt/homebrew/opt/qt@5/lib/pkgconfig:${PKG_CONFIG_PATH}"
export PATH="/opt/homebrew/opt/arm-gcc-bin@13/bin:$PATH"
```

## 7. Build and smoke-test SITL

```bash
cd ~/aeris-deps/PX4-Autopilot
source .venv/bin/activate     # PX4's own build venv, created by macos.sh
HEADLESS=1 make px4_sitl gz_x500          # base smoke test
HEADLESS=1 make px4_sitl gz_x500_depth    # depth + RGB camera
HEADLESS=1 make px4_sitl gz_x500_lidar_2d # 2D LiDAR
```

Each command builds (first run only, ~10–15 min on this Mac) and then
**launches and runs** the simulator + PX4 in the foreground. You should see:

```
INFO  [init] Gazebo world is ready
INFO  [init] Spawning Gazebo model
INFO  [px4] Startup script returned successfully
pxh>
```

Verify sensor data from another terminal (with the same env sourced from
`build/px4_sitl_default/rootfs/gz_env.sh`, or `GZ_IP=127.0.0.1` set):

```bash
source build/px4_sitl_default/rootfs/gz_env.sh
GZ_IP=127.0.0.1 gz topic -l                          # list active topics
GZ_IP=127.0.0.1 gz topic -e -t /depth_camera -n 1    # one depth frame
```

Confirmed working on this machine (Phase 1): `gz_x500_depth`'s
`/depth_camera` and RGB `.../IMX214/image`, and `gz_x500_lidar_2d`'s
`.../lidar_2d_v2/scan` all publish real, incrementing data headless, with
no `--render-engine ogre` (Ogre1) fallback needed — Ogre2 works natively.

## 8. Vehicle telemetry

`mavsdk` (Python) 4.0.0's native binding currently **segfaults on
construction** on this machine — see `configs/versions.lock.yaml` →
`mavsdk_python`. Use `pymavlink` instead until Phase 4 re-evaluates:

```bash
python -c "
from pymavlink import mavutil
conn = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
print(conn.wait_heartbeat(timeout=10))
"
```

## 9. QGroundControl (optional, debug tool only)

Not installed in Phase 1 — not on the headless critical path. To install:
download the QGroundControl 5.x `.dmg` from https://qgroundcontrol.com,
mount it, and drag the app to `/Applications`. macOS 13+ required (this
machine is well above that).

## Disk / time budget actually used

- PX4 clone + submodules: **2.7 GB**
- Homebrew formulas (toolchain + Gazebo Harmonic + deps): **~1.5–2 GB**
- First `make px4_sitl gz_x500` full build: **~10–15 minutes** wall time
  (bottled dependencies, no source compiles except PX4 itself and the
  small `PX4-OpticalFlow` external project)
