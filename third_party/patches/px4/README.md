# PX4 macOS compatibility patches (Phase 1)

These patches are **build-system / toolchain compatibility fixes only**.
None of them change PX4 flight-control, estimation, or simulation *behavior*.
They exist because compiling PX4 v1.17.0's Gazebo integration against the
current (2026-09) Homebrew toolchain on macOS 27.2 / Apple clang 21 surfaces
a handful of `-Werror`-fatal warnings and two build-portability bugs that
PX4's own macOS CI apparently does not exercise in this exact combination.

Applied against **PX4-Autopilot tag `v1.17.0`**, commit
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.

Apply with, from the PX4-Autopilot repo root:

```bash
for p in third_party/patches/px4/*.patch; do git apply "$p"; done
```

(or `patch -p1 < file.patch` per file if not using a git checkout).

## Patch index

| # | File(s) touched | Symptom fixed |
|---|---|---|
| 0001 | `gz_plugins/moving_platform_controller/MovingPlatformController.cpp` | `-Wdouble-promotion -Werror`: implicit `float`→`double` in a `gz::math::Vector3d(...)` constructor call. This is the same *class* of bug originally reported in PX4/PX4-Autopilot#27026 (closed 2026-04-21), recurring in a newer file. Fixed with explicit `static_cast<double>`. |
| 0002 | `gz_msgs/CMakeLists.txt` | Modern protobuf (≥26) pulls in Abseil headers that hard-error below C++17 (`"C++ versions less than C++17 are not supported."`). This target had no explicit C++ standard and inherited the project's older default (`gnu++14`). Fixed with `target_compile_features(px4_gz_msgs PUBLIC cxx_std_17)`. |
| 0004a | `gz_bridge/GZMixingInterfaceServo.cpp` | `-Wdouble-promotion`: `float`-returning getters assigned to `double` locals without a cast. |
| 0004b | `gz_bridge/GZGimbal.cpp` | `-Wdouble-promotion`: `float` passed to protobuf's `set_data(double)`. |
| 0005a | `gz_bridge/GZMixingInterfaceWheel.cpp` | `RepeatedField::Resize()` deprecated in the pinned protobuf (`ABSL_DEPRECATE_AND_INLINE`); renamed to the still-supported lowercase `resize()`. |
| 0005b | `gz_bridge/GZMixingInterfaceESC.cpp` | Same as 0005a — this is the exact file/symptom pairing originally reported in #27026 (that report's symptom was a *different* warning in the same file; this is a new one in the same spot). |
| 0006 | `gz_plugins/optical_flow/OpticalFlowSensor.hpp` | `-Wunused-private-field`: `_horizontal_fov`/`_vertical_fov` are declared but never read anywhere in the class. Marked `[[maybe_unused]]` rather than removed, to avoid silently changing the class layout/intent. |
| 0007 | `gz_plugins/gstreamer/CMakeLists.txt` | Linker: `library 'gstreamer-1.0' not found`. `pkg_check_modules(GSTREAMER gstreamer-1.0)` populates `GSTREAMER_LIBRARY_DIRS` separately from `GSTREAMER_LIBRARIES` (bare `-l` names, no path); this file only ever consumed the latter. A no-op on systems where GStreamer happens to sit on the default linker search path; not the case on Homebrew macOS. Fixed by adding `target_link_directories(... ${GSTREAMER_LIBRARY_DIRS} ${GSTREAMER_APP_LIBRARY_DIRS})`. |
| 0008 | `gz_plugins/optical_flow/optical_flow.cmake` | `ExternalProject_Add(OpticalFlow ...)`'s `BUILD_BYPRODUCTS`/`OpticalFlow_LIBS` paths were hardcoded to the `.so` (ELF) suffix. CMake's own build of that same external project correctly produces `libOpticalFlow.dylib` on Darwin — only these two hardcoded path strings didn't match. Fixed with `${CMAKE_SHARED_LIBRARY_SUFFIX}` (portable: `.so` on Linux, unchanged; `.dylib` on macOS). |

## Not a source patch: protobuf 36.1 / 36.2 dylib coexistence

Separately from these patches, this Mac's Homebrew Cellar has **two protobuf
minor-version `.dylib` files present in one linked keg**:

- `Cellar/protobuf/36.1/lib/libprotobuf.36.1.0.dylib` — the *installed and
  linked* protobuf. Pinned to exactly 36.1 (not `brew install`'s current
  36.2) because `gz-sim8` and friends' bottled `.dylib`s were built against
  36.1 by SONAME.
- `Cellar/protobuf/36.1/lib/libprotobuf.36.2.0.dylib` — extracted from the
  **current** (36.2) protobuf bottle via `brew fetch` + manual `tar`, then
  `install_name_tool -id`/`-change`'d from the raw bottle's
  `@@HOMEBREW_PREFIX@@` placeholder to the real `/opt/homebrew` path and
  re-signed (`codesign --force --sign -`), and copied alongside 36.1's own
  files. This exists because `opencv@4`'s bottled `libopencv_dnn.dylib` (an
  indirect dependency of PX4's `OpticalFlow` plugin, via `PX4-OpticalFlow`'s
  own unrestricted `find_package(OpenCV REQUIRED)`) was built against 36.2,
  and Homebrew cannot hold two *installed formula* versions of protobuf
  linked simultaneously.

This is **environment state, not a PX4 source change** — there is nothing to
`git apply`. It must be reproduced by script or documented manually on any
other machine. See `configs/versions.lock.yaml` → `protobuf` /
`known_source_patches.manual_workaround_not_yet_patched_as_file` for the
full rationale, and treat it as a known fragility: a future `brew upgrade`
of either `protobuf` or `opencv@4` can silently break this again.

## Also required (Homebrew-side, no source patch)

- `Tools/setup/macos.sh`, `gz-tap-pin.txt`, `protobuf-pin.txt`,
  `requirements.txt` under `Tools/setup/` were replaced with the versions
  from `origin/main` (the v1.17.0 tag's copies call the now-deprecated
  no-op `px4-dev`/`px4-sim` Homebrew meta-formulae and don't handle
  Homebrew 6.0+'s tap-trust requirement). Originals preserved at
  `Tools/setup/_v1.17.0_original/` inside the PX4 checkout itself (not
  copied into this AERIS repo, since PX4 is an external dependency per
  ADR-002).
- `opencv@4` and `qt@5` are Homebrew keg-only formulae and must be added to
  `CMAKE_PREFIX_PATH` / `LDFLAGS` / `CPPFLAGS` / `PKG_CONFIG_PATH` explicitly
  before building. See `docs/mac-setup.md` for the exact env block.
