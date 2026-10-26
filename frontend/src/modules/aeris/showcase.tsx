"use client";
import { lazy, Suspense, useEffect, useId, useRef, useState } from "react";
import {
  AnimatePresence,
  MotionConfig,
  m,
  LazyMotion,
  useReducedMotion,
  useSpring,
  useTransform,
} from "motion/react";
import {
  Activity,
  ArrowDownToLine,
  ArrowRight,
  ArrowUpRight,
  Battery,
  Check,
  ChevronDown,
  CircleHelp,
  Expand,
  FlaskConical,
  Focus,
  Gauge,
  Layers,
  Map,
  Moon,
  MoveUpRight,
  Navigation,
  Pause,
  Play,
  Radio,
  RotateCcw,
  ScanLine,
  Search,
  ShieldCheck,
  Sun,
  Target,
  Timer,
  Triangle,
  X,
  ZoomIn,
  ZoomOut,
} from "lucide-react";
import { ShowcaseBoundary } from "./boundary";
import { StoreProvider, useAeris, type View } from "./store";
import {
  formatTime,
  positionAt,
  recordings,
  scenarios,
  snapshot,
  type Frame,
  type Scenario,
} from "./data";
import { Button } from "./ui/button";
import s from "./showcase.module.css";

// The initial screen is fully server-rendered. Animation/layout features are
// progressive enhancement and are fetched on the first real interaction.
const loadMotionFeatures = () =>
  new Promise<typeof import("./motion-features").default>((resolve) => {
    const load = () => {
      window.removeEventListener("pointerdown", load);
      window.removeEventListener("keydown", load);
      void import("./motion-features").then((module) =>
        resolve(module.default),
      );
    };
    window.addEventListener("pointerdown", load, { once: true, passive: true });
    window.addEventListener("keydown", load, { once: true });
  });
const CommandPalette = lazy(() => import("./command-palette"));
const HelpDialog = lazy(() => import("./help-dialog"));

export function AerisShowcase({ demo = true }: { demo?: boolean }) {
  return (
    <ShowcaseBoundary>
      <StoreProvider>
        <LazyMotion features={loadMotionFeatures}>
          <MotionConfig reducedMotion="user">
            <Experience demo={demo} />
          </MotionConfig>
        </LazyMotion>
      </StoreProvider>
    </ShowcaseBoundary>
  );
}
function Experience({ demo }: { demo: boolean }) {
  const a = useAeris((v) => v);
  const rootRef = useRef<HTMLDivElement>(null);
  const contentId = useId();
  const [palette, setPalette] = useState(false);
  const [help, setHelp] = useState(false);
  const [notice, setNotice] = useState("");
  const [loading, setLoading] = useState(false);
  const scenario = scenarios.find((v) => v.id === a.scenario)!;
  const frame = recordings[scenario.id][Math.floor(a.time)];
  useEffect(() => {
    if (!a.playing || !demo) return;
    const timer = setInterval(a.tick, 250);
    return () => clearInterval(timer);
  }, [a.playing, a.tick, demo]);
  useEffect(() => {
    if (!notice) return;
    const timer = setTimeout(() => setNotice(""), 4000);
    return () => clearTimeout(timer);
  }, [notice]);
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.defaultPrevented) return;
      const owner =
        e.target instanceof HTMLElement
          ? e.target.closest("[data-aeris-showcase]")
          : null;
      if (owner && owner !== rootRef.current) return;
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "k") {
        e.preventDefault();
        setPalette((p) => !p);
        return;
      }
      if (
        palette ||
        help ||
        !demo ||
        (e.target instanceof HTMLElement &&
          (e.target.matches("input, select, textarea, button") ||
            e.target.isContentEditable))
      )
        return;
      if (e.code === "Space") {
        e.preventDefault();
        a.toggle();
      }
      if (e.key === "ArrowRight") {
        e.preventDefault();
        a.seek(a.time + 10);
      }
      if (e.key === "ArrowLeft") {
        e.preventDefault();
        a.seek(a.time - 10);
      }
      if (e.key === "?") setHelp(true);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [a, palette, help, demo]);
  function selectScenario(id: string) {
    a.setScenario(id);
    setLoading(true);
  }
  useEffect(() => {
    if (!loading) return;
    const t = setTimeout(() => setLoading(false), 240);
    return () => clearTimeout(t);
  }, [loading]);
  function download() {
    const blob = new Blob(
      [JSON.stringify(snapshot(scenario, a.time), null, 2)],
      { type: "application/json" },
    );
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `aeris-${scenario.id}-${Math.floor(a.time)}s.json`;
    link.click();
    URL.revokeObjectURL(url);
    setNotice("Snapshot exported · synthetic demo data");
  }
  const tabs: { id: View; label: string; icon: typeof Map }[] = [
    { id: "mission", label: "Mission view", icon: Map },
    { id: "analysis", label: "Analytics", icon: Activity },
    { id: "runs", label: "Mission library", icon: Layers },
  ];
  return (
    <div
      ref={rootRef}
      className={s.root}
      data-theme={a.theme}
      data-aeris-showcase
    >
      <a href={`#${contentId}`} className={s.skip}>
        Skip to mission content
      </a>
      <header className={s.header}>
        <a
          className={s.brand}
          href={`#${contentId}`}
          aria-label="AERIS Autonomy Lab"
        >
          <span className={s.brandMark}>
            <Triangle size={25} strokeWidth={1.6} />
            <i />
          </span>
          AERIS<span className={s.brandCaption}>AUTONOMY LAB</span>
        </a>
        <div className={s.headerCenter}>
          <span className={s.statusDot} />
          Systems worth exploring.
        </div>
        <div className={s.headerActions}>
          <span className={s.demoPill}>
            <FlaskConical size={12} />
            Interactive demo
          </span>
          <Button
            className={s.commandTrigger}
            variant="ghost"
            onClick={() => setPalette(true)}
            aria-label="Quick actions: open command palette"
          >
            <Search size={16} />
            <span>Quick actions</span>
            <kbd>⌘ K</kbd>
          </Button>
          <Button
            variant="ghost"
            size="icon"
            aria-label={`Switch to ${a.theme === "light" ? "dark" : "light"} theme`}
            onClick={() => a.setTheme(a.theme === "light" ? "dark" : "light")}
          >
            {a.theme === "light" ? <Moon size={17} /> : <Sun size={17} />}
          </Button>
        </div>
      </header>
      <div className={s.shell}>
        <aside className={s.sidebar}>
          <div className={s.workspace}>
            <span className={s.workspaceIcon}>
              <Navigation size={18} />
            </span>
            <div>
              <strong>Research workspace</strong>
              <small>Personal environment</small>
            </div>
          </div>
          <div className={s.navLabel}>WORKSPACE</div>
          <nav aria-label="Workspace">
            {tabs.map((t) => (
              <button
                className={`${s.navItem} ${a.view === t.id ? s.navActive : ""}`}
                key={t.id}
                onClick={() => a.setView(t.id)}
                aria-current={a.view === t.id ? "page" : undefined}
              >
                <t.icon size={17} />
                {t.label}
                {a.view === t.id && (
                  <m.span layoutId="nav-active" className={s.navIndicator} />
                )}
              </button>
            ))}
          </nav>
          <div className={s.sidebarSection}>
            <div className={s.navLabel}>
              EXPLORE A WORLD <span>03</span>
            </div>
            {scenarios.map((v, i) => (
              <button
                onClick={() => selectScenario(v.id)}
                key={v.id}
                aria-pressed={v.id === a.scenario}
                className={`${s.scenarioItem} ${v.id === a.scenario ? s.scenarioSelected : ""}`}
              >
                <span className={s.scenarioNumber}>0{i + 1}</span>
                <span>
                  {v.name}
                  <small>{v.family.split(" · ")[0]} environment</small>
                </span>
                {v.id === a.scenario && <span className={s.tinyDot} />}
              </button>
            ))}
          </div>
          <div className={s.sidebarBottom}>
            <div className={s.safetyNote}>
              <ShieldCheck size={19} />
              <strong>A safe space to explore.</strong>
              <p>Every control stays in this demo. No vehicle is connected.</p>
              <button onClick={() => setHelp(true)}>
                How it works <ArrowUpRight size={13} />
              </button>
            </div>
            <button className={s.helpButton} onClick={() => setHelp(true)}>
              <CircleHelp size={16} />
              Guide & shortcuts<kbd>?</kbd>
            </button>
            <span className={s.version}>AERIS / SHOWCASE 01</span>
          </div>
        </aside>
        <main id={contentId} className={s.main}>
          <div className={s.breadcrumb}>
            Workspace <span>/</span> {tabs.find((t) => t.id === a.view)?.label}
            <span className={s.breadcrumbRight}>
              <span className={s.statusDot} /> LOCAL PLAYBACK
            </span>
          </div>
          <div className={s.pageHeading}>
            <div>
              <div className={s.eyebrow}>PERCEIVE. NAVIGATE. EXPLORE.</div>
              <h1>
                {a.view === "mission"
                  ? "Autonomy, observed."
                  : a.view === "analysis"
                    ? "Every move, understood."
                    : "A world of possibilities."}
              </h1>
              <p>
                {a.view === "mission"
                  ? "Follow the mission. Find the signal in every movement."
                  : a.view === "analysis"
                    ? "Connect the route to the results. All examples are synthetic."
                    : "Choose an environment and explore its illustrative mission."}
              </p>
            </div>
            <div className={s.headingActions}>
              <Button
                variant="ghost"
                size="icon"
                aria-label="About this demo"
                onClick={() => setHelp(true)}
              >
                <CircleHelp size={18} />
              </Button>
              <Button onClick={download} disabled={!demo}>
                <ArrowDownToLine size={15} />
                <span>Export snapshot</span>
              </Button>
            </div>
          </div>
          <div className={s.sourceBar}>
            <span>
              <FlaskConical size={14} />
              <strong>DEMO MODE</strong>
              <span>Seeded simulation · not measured research</span>
            </span>
            <button onClick={() => setHelp(true)}>
              About the data <ArrowUpRight size={13} />
            </button>
          </div>
          {!demo ? (
            <div className={s.disconnected} role="status">
              <Radio size={32} />
              <h2>No data source connected</h2>
              <p>
                The live API is not implemented. Enable NEXT_PUBLIC_DEMO_MODE to
                explore the local fixture.
              </p>
            </div>
          ) : (
            <AnimatePresence mode="wait">
              <m.div
                key={a.view}
                initial={false}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, y: -4 }}
                transition={{ duration: 0.18 }}
              >
                {a.view === "mission" ? (
                  <>
                    <m.section
                      initial="hidden"
                      animate="show"
                      variants={{
                        hidden: {},
                        show: { transition: { staggerChildren: 0.055 } },
                      }}
                      className={s.metrics}
                      aria-label="Mission telemetry"
                    >
                      <Metric
                        label="Area explored"
                        value={frame.coverage.toFixed(1)}
                        unit="%"
                        icon={ScanLine}
                        caption="of accessible floor area"
                        values={recordings[scenario.id]
                          .slice(0, Math.floor(a.time) + 1)
                          .map((f) => f.coverage)}
                      />
                      <Metric
                        label="Distance traveled"
                        value={frame.distance.toFixed(1)}
                        unit="m"
                        icon={Navigation}
                        caption="cumulative path length"
                        values={recordings[scenario.id]
                          .slice(0, Math.floor(a.time) + 1)
                          .map((f) => f.distance)}
                      />
                      <Metric
                        label="Battery remaining"
                        value={frame.battery.toFixed(0)}
                        unit="%"
                        icon={Battery}
                        caption="illustrative discharge model"
                        values={recordings[scenario.id]
                          .slice(0, Math.floor(a.time) + 1)
                          .map((f) => f.battery)}
                      />
                      <Metric
                        label="Mission time"
                        value={formatTime(a.time)}
                        unit=""
                        icon={Timer}
                        caption={`of ${formatTime(scenario.duration)} recorded`}
                        values={[]}
                      />
                    </m.section>
                    <div className={s.missionGrid}>
                      <section className={s.mapPanel} aria-label="Mission map">
                        <div className={s.panelHeader}>
                          <div className={s.panelTitle}>
                            <span className={s.limeSquare}>
                              <Map size={15} />
                            </span>
                            <h2>Mission canvas</h2>
                            <span className={s.tag}>2D / ENU</span>
                          </div>
                          <label className={s.selectWrap}>
                            <span className={s.srOnly}>Environment</span>
                            <select
                              aria-label="Environment"
                              value={a.scenario}
                              onChange={(e) => selectScenario(e.target.value)}
                            >
                              {scenarios.map((v) => (
                                <option key={v.id} value={v.id}>
                                  {v.name}
                                </option>
                              ))}
                            </select>
                            <ChevronDown size={13} />
                          </label>
                        </div>
                        {loading ? (
                          <div
                            className={s.mapSkeleton}
                            role="status"
                            aria-label="Loading scenario"
                          >
                            <div />
                            <div />
                            <span>Preparing seeded world…</span>
                          </div>
                        ) : (
                          <MissionMap scenario={scenario} frame={frame} />
                        )}
                        <Timeline scenario={scenario} />
                      </section>
                      <MissionDetails scenario={scenario} frame={frame} />
                    </div>
                    <div className={s.bottomGrid}>
                      <Telemetry scenario={scenario} />
                      <div className={s.insightPanel}>
                        <div className={s.eyebrow}>
                          <Focus size={14} />
                          THE BIGGER PICTURE
                        </div>
                        <h2>
                          A path becomes <br />
                          an understanding.
                        </h2>
                        <p>
                          Scrub the timeline to see how exploration builds a
                          map, one observation at a time.
                        </p>
                        <button onClick={() => a.setView("analysis")}>
                          Explore mission analytics <ArrowRight size={16} />
                        </button>
                        <div className={s.insightLines}>
                          <span />
                          <span />
                          <span />
                        </div>
                      </div>
                    </div>
                  </>
                ) : a.view === "analysis" ? (
                  <Analytics scenario={scenario} />
                ) : (
                  <Library
                    onSelect={(id) => {
                      selectScenario(id);
                      a.setView("mission");
                    }}
                  />
                )}
              </m.div>
            </AnimatePresence>
          )}
          <footer className={s.footer}>
            <span>
              <Triangle size={12} /> Built for curiosity. Grounded in evidence.
            </span>
            <span>
              ILLUSTRATIVE DATA <span> / </span> SEED {scenario.seed}
            </span>
          </footer>
        </main>
      </div>
      <Suspense fallback={null}>
        {palette && (
          <CommandPalette
            open={palette}
            onOpenChange={setPalette}
            theme={a.theme}
            demo={demo}
            onView={a.setView}
            onScenario={selectScenario}
            onToggle={a.toggle}
            onTheme={() => a.setTheme(a.theme === "light" ? "dark" : "light")}
            onExport={download}
          />
        )}
        {help && (
          <HelpDialog open={help} onOpenChange={setHelp} theme={a.theme} />
        )}
      </Suspense>
      <AnimatePresence>
        {notice && (
          <m.div
            className={s.toast}
            role="status"
            initial={{ opacity: 0, y: 16 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0 }}
          >
            <Check size={17} />
            {notice}
            <button
              aria-label="Dismiss notification"
              onClick={() => setNotice("")}
            >
              <X size={14} />
            </button>
          </m.div>
        )}
      </AnimatePresence>
    </div>
  );
}
function Metric({
  label,
  value,
  unit,
  icon: Icon,
  caption,
  values,
}: {
  label: string;
  value: string;
  unit: string;
  icon: typeof Map;
  caption: string;
  values: number[];
}) {
  return (
    <m.div className={s.metric} variants={{ hidden: { y: 8 }, show: { y: 0 } }}>
      <div className={s.metricLabel}>
        {label}
        <Icon size={16} />
      </div>
      <div className={s.metricValue}>
        {unit ? <AnimatedNumber value={value} /> : <span>{value}</span>}
        <span>{unit}</span>
        {values.length > 1 && <Sparkline values={values} />}
      </div>
      <small>{caption}</small>
    </m.div>
  );
}
function AnimatedNumber({ value }: { value: string }) {
  const reduced = useReducedMotion();
  const spring = useSpring(Number(value), { stiffness: 180, damping: 28 });
  const digits = value.includes(".") ? value.split(".")[1].length : 0;
  const display = useTransform(spring, (n) => n.toFixed(digits));
  useEffect(() => {
    if (reduced) spring.jump(Number(value));
    else spring.set(Number(value));
  }, [spring, value, reduced]);
  return (
    <span>
      <m.span aria-hidden="true">{display}</m.span>
      <span className={s.srOnly}>{value}</span>
    </span>
  );
}
function Sparkline({ values }: { values: number[] }) {
  const low = Math.min(...values);
  const range = Math.max(...values) - low || 1;
  return (
    <svg aria-hidden="true" className={s.sparkline} viewBox="0 0 90 30">
      <path
        d={values
          .map(
            (v, i) =>
              `${i ? "L" : "M"}${(i / (values.length - 1)) * 88 + 1},${27 - ((v - low) / range) * 23}`,
          )
          .join(" ")}
        fill="none"
        stroke="currentColor"
        strokeWidth="1.8"
      />
    </svg>
  );
}
function MissionMap({ scenario, frame }: { scenario: Scenario; frame: Frame }) {
  const a = useAeris((v) => v);
  const [zoom, setZoom] = useState(1);
  const id = useId().replaceAll(":", "");
  const reduced = useReducedMotion();
  const route = recordings[scenario.id]
    .slice(0, Math.floor(a.time) + 1)
    .map((p) => `${p.x},${p.y}`)
    .join(" ");
  const p = positionAt(scenario, a.time);
  return (
    <div className={s.mapStage}>
      <div className={s.mapTop}>
        <span className={s.mapBadge}>
          <span className={s.statusDot} />
          {a.playing ? "PLAYING REPLAY" : "REPLAY PAUSED"}
        </span>
        <span className={s.mapSeed}>WORLD / {scenario.seed}</span>
      </div>
      <div className={s.mapViewport}>
        <svg
          viewBox={`${360 - 360 / zoom} ${220 - 220 / zoom} ${720 / zoom} ${440 / zoom}`}
          className={s.floorplan}
          role="img"
          aria-label={`${scenario.name} local ENU floorplan. Vehicle at east ${(p.x / 10).toFixed(1)} meters, north ${((440 - p.y) / 10).toFixed(1)} meters. ${frame.coverage.toFixed(1)} percent explored.`}
        >
          <defs>
            <pattern
              id={`grid-${id}`}
              width="20"
              height="20"
              patternUnits="userSpaceOnUse"
            >
              <path
                d="M 20 0 L 0 0 0 20"
                fill="none"
                stroke="var(--map-grid)"
                strokeWidth=".7"
              />
            </pattern>
            <pattern
              id={`hatch-${id}`}
              width="6"
              height="6"
              patternUnits="userSpaceOnUse"
              patternTransform="rotate(45)"
            >
              <line y2="6" stroke="var(--obstacle-line)" strokeWidth="1.5" />
            </pattern>
            <clipPath id={`room-${id}`}>
              <rect x="40" y="40" width="640" height="360" rx="3" />
            </clipPath>
          </defs>
          <rect width="720" height="440" fill={`url(#grid-${id})`} />
          <g clipPath={`url(#room-${id})`}>
            <rect x="40" y="40" width="640" height="360" fill="var(--room)" />
            {a.scan && (
              <polyline
                points={route}
                fill="none"
                stroke="var(--scan)"
                strokeWidth="166"
                strokeLinejoin="round"
                strokeLinecap="round"
                opacity=".35"
              />
            )}
            <rect
              x="40"
              y="40"
              width="640"
              height="360"
              fill={`url(#grid-${id})`}
            />
          </g>
          <rect
            x="40"
            y="40"
            width="640"
            height="360"
            rx="3"
            fill="none"
            stroke="var(--wall)"
            strokeWidth="5"
          />
          {scenario.obstacles.map((o, i) => (
            <g key={i}>
              <rect
                x={o.x + 4}
                y={o.y + 4}
                width={o.w}
                height={o.h}
                fill="var(--obstacle-shadow)"
                rx="2"
              />
              <rect
                x={o.x}
                y={o.y}
                width={o.w}
                height={o.h}
                fill="var(--obstacle)"
                stroke="var(--obstacle-line)"
                strokeWidth="1.3"
                rx="2"
              />
              <rect
                x={o.x}
                y={o.y}
                width={o.w}
                height={o.h}
                fill={`url(#hatch-${id})`}
                opacity=".55"
              />
              <text
                x={o.x + o.w / 2}
                y={o.y + o.h / 2 + 4}
                textAnchor="middle"
                className={s.mapText}
              >
                {scenario.id === "rubble" ? "R" : "B"}
                {String(i + 1).padStart(2, "0")}
              </text>
            </g>
          ))}
          {a.route && (
            <>
              <polyline
                points={scenario.route.map((p) => `${p.x},${p.y}`).join(" ")}
                fill="none"
                stroke="var(--planned)"
                strokeWidth="1.5"
                strokeDasharray="4 6"
              />
              <polyline
                points={route}
                fill="none"
                stroke="var(--route)"
                strokeWidth="2.5"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
              {scenario.route.slice(0, -1).map((v, i) => (
                <circle
                  key={i}
                  cx={v.x}
                  cy={v.y}
                  r="3"
                  fill="var(--room)"
                  stroke="var(--route)"
                  strokeWidth="1.5"
                />
              ))}
            </>
          )}
          <circle
            cx="85"
            cy="355"
            r="9"
            fill="var(--room)"
            stroke="var(--route)"
            strokeWidth="1.5"
          />
          <text x="85" y="359" textAnchor="middle" className={s.mapHome}>
            H
          </text>
          {scenario.targets.map((v, i) => (
            <g key={i} opacity={a.time >= v.time ? 1 : 0.45}>
              <circle
                cx={v.x}
                cy={v.y}
                r="14"
                fill="var(--target-bg)"
                stroke="var(--target)"
                strokeDasharray={a.time < v.time ? "3 3" : undefined}
              />
              <text
                x={v.x}
                y={v.y + 4}
                textAnchor="middle"
                className={s.mapTarget}
              >
                {String.fromCharCode(65 + i)}
              </text>
            </g>
          ))}
          {a.scan && (
            <>
              <circle
                cx={p.x}
                cy={p.y}
                r="56"
                fill="none"
                stroke="var(--route)"
                opacity=".2"
                strokeDasharray="3 4"
              />
              <m.circle
                cx={p.x}
                cy={p.y}
                r="30"
                fill="var(--route)"
                initial={false}
                animate={
                  a.playing && !reduced
                    ? { opacity: [0.04, 0.15, 0.04], r: [24, 40, 24] }
                    : { opacity: 0.08, r: 30 }
                }
                transition={{
                  duration: 2,
                  repeat: a.playing && !reduced ? Infinity : 0,
                }}
              />
            </>
          )}
          <g transform={`translate(${p.x}, ${p.y})`}>
            <circle
              r="13"
              fill="var(--ink)"
              stroke="var(--surface)"
              strokeWidth="3"
            />
            <path d="M0 -7 L5 5 L0 2 L-5 5 Z" fill="var(--accent)" />
            <g transform="translate(18,-33)">
              <rect width="105" height="26" rx="4" fill="var(--ink)" />
              <text
                x="9"
                y="17"
                fill="var(--surface)"
                fontSize="10"
                fontFamily="monospace"
              >
                AERIS / 01
              </text>
            </g>
          </g>
          <text x="365" y="28" textAnchor="middle" className={s.mapAxis}>
            64 m · EAST →
          </text>
          <text
            x="17"
            y="220"
            transform="rotate(-90 17 220)"
            textAnchor="middle"
            className={s.mapAxis}
          >
            36 m · NORTH →
          </text>
        </svg>
      </div>
      <div className={s.mapControls}>
        <div className={s.layerControls}>
          <button aria-pressed={a.route} onClick={() => a.setLayer("route")}>
            <Navigation size={13} />
            Route
          </button>
          <button aria-pressed={a.scan} onClick={() => a.setLayer("scan")}>
            <ScanLine size={13} />
            Sensor footprint
          </button>
        </div>
        <div className={s.zoomControls}>
          <Button
            size="icon"
            aria-label="Zoom out"
            onClick={() => setZoom((v) => Math.max(1, v - 0.2))}
            disabled={zoom <= 1}
          >
            <ZoomOut size={15} />
          </Button>
          <Button
            size="icon"
            aria-label="Reset map view"
            onClick={() => setZoom(1)}
          >
            <Expand size={14} />
          </Button>
          <Button
            size="icon"
            aria-label="Zoom in"
            onClick={() => setZoom((v) => Math.min(1.8, v + 0.2))}
            disabled={zoom >= 1.8}
          >
            <ZoomIn size={15} />
          </Button>
        </div>
      </div>
      <div className={s.mapLegend}>
        <span>
          <i className={s.exploredLegend} />
          Observed area
        </span>
        <span>
          <i className={s.routeLegend} />
          Flight path
        </span>
        <span>
          <i className={s.targetLegend} />
          Inspection point
        </span>
        <span className={s.scale}>10 m ━━━</span>
      </div>
    </div>
  );
}
function Timeline({ scenario }: { scenario: Scenario }) {
  const speedId = useId();
  const a = useAeris((v) => v);
  return (
    <div className={s.timeline}>
      <div className={s.timelineTop}>
        <div className={s.playback}>
          <Button
            size="icon"
            variant="default"
            aria-label={a.playing ? "Pause replay" : "Play replay"}
            onClick={a.toggle}
          >
            {a.playing ? (
              <Pause size={16} fill="currentColor" />
            ) : (
              <Play size={16} fill="currentColor" />
            )}
          </Button>
          <Button
            size="icon"
            variant="ghost"
            aria-label="Restart replay"
            onClick={() => a.seek(0)}
          >
            <RotateCcw size={15} />
          </Button>
          <span className={s.clock}>
            {formatTime(a.time)} <span>/ {formatTime(scenario.duration)}</span>
          </span>
        </div>
        <div className={s.speed}>
          <label htmlFor={speedId}>Playback</label>
          <select
            id={speedId}
            aria-label="Playback"
            value={a.speed}
            onChange={(e) => a.setSpeed(+e.target.value)}
          >
            <option value=".5">0.5×</option>
            <option value="1">1×</option>
            <option value="2">2×</option>
            <option value="4">4×</option>
          </select>
        </div>
      </div>
      <input
        className={s.scrubber}
        aria-label="Mission timeline"
        aria-valuetext={`${formatTime(a.time)} of ${formatTime(scenario.duration)}`}
        type="range"
        min="0"
        max={scenario.duration}
        step="1"
        value={a.time}
        onChange={(e) => a.seek(+e.target.value)}
        style={
          {
            "--progress": `${(a.time / scenario.duration) * 100}%`,
          } as React.CSSProperties
        }
      />
      <div className={s.timelineTicks}>
        <span>00:00</span>
        <span>01:00</span>
        <span>02:00</span>
        <span>03:00</span>
        <span>04:00</span>
      </div>
    </div>
  );
}
function MissionDetails({
  scenario,
  frame,
}: {
  scenario: Scenario;
  frame: Frame;
}) {
  const a = useAeris((v) => v);
  const [filter, setFilter] = useState("all");
  const events = scenario.events
    .filter((e) => e.time <= a.time && (filter === "all" || e.kind === filter))
    .toReversed();
  return (
    <aside className={s.details}>
      <section className={s.vehicle}>
        <div className={s.panelHeader}>
          <h2>Vehicle overview</h2>
          <span className={s.tag}>SIMULATED</span>
        </div>
        <div className={s.droneCard}>
          <div className={s.droneDrawing}>
            <svg
              width="130"
              height="90"
              viewBox="0 0 130 90"
              aria-hidden="true"
            >
              <g fill="none" stroke="currentColor" strokeWidth="1.5">
                <path d="M40 20 L90 70 M90 20 L40 70" strokeWidth="7" />
                <rect
                  x="53"
                  y="30"
                  width="24"
                  height="29"
                  rx="6"
                  fill="var(--surface)"
                />
                <circle cx="37" cy="18" r="15" />
                <circle cx="93" cy="18" r="15" />
                <circle cx="37" cy="72" r="15" />
                <circle cx="93" cy="72" r="15" />
                <circle cx="37" cy="18" r="3" />
                <circle cx="93" cy="18" r="3" />
                <circle cx="37" cy="72" r="3" />
                <circle cx="93" cy="72" r="3" />
              </g>
              <path
                d="M61 40 L65 35 L69 40"
                stroke="var(--green)"
                fill="none"
                strokeWidth="2"
              />
            </svg>
          </div>
          <div>
            <strong>AERIS / 01</strong>
            <small>Quadrotor · X500 profile</small>
            <span className={s.nominal}>
              <span className={s.tinyDot} /> Example telemetry
            </span>
          </div>
        </div>
        <div className={s.vehicleStats}>
          <span>
            <MoveUpRight size={14} />
            Altitude
            <strong>
              {frame.altitude.toFixed(1)} <small>m</small>
            </strong>
          </span>
          <span>
            <Gauge size={14} />
            Velocity
            <strong>
              {frame.speed.toFixed(1)} <small>m/s</small>
            </strong>
          </span>
        </div>
      </section>
      <section className={s.objectives}>
        <div className={s.panelHeader}>
          <h2>Inspection points</h2>
          <span className={s.tag}>
            {scenario.targets.filter((t) => t.time <= a.time).length} / 2
          </span>
        </div>
        {scenario.targets.map((v, i) => (
          <button
            className={s.objective}
            key={v.name}
            onClick={() => a.seek(v.time)}
            title={`Jump to ${v.name}`}
          >
            <span
              className={
                v.time <= a.time ? s.objectiveDone : s.objectivePending
              }
            >
              {v.time <= a.time ? <Check size={15} /> : <Target size={15} />}
            </span>
            <span>
              <strong>Point {String.fromCharCode(65 + i)}</strong>
              <small>
                {v.time <= a.time ? "Reached" : "Upcoming"} ·{" "}
                {formatTime(v.time)}
              </small>
            </span>
            <ArrowUpRight size={14} />
          </button>
        ))}
      </section>
      <section className={s.events}>
        <div className={s.panelHeader}>
          <h2>Mission events</h2>
          <label>
            <span className={s.srOnly}>Filter events</span>
            <select
              aria-label="Filter events"
              value={filter}
              onChange={(e) => setFilter(e.target.value)}
            >
              <option value="all">All events</option>
              <option value="target">Inspection</option>
              <option value="navigation">Navigation</option>
              <option value="system">System</option>
            </select>
          </label>
        </div>
        <div className={s.eventList}>
          {events.length ? (
            <AnimatePresence initial={false}>
              {events.slice(0, 4).map((e) => (
                <m.button
                  layout
                  key={e.time}
                  initial={{ opacity: 0, x: 8 }}
                  animate={{ opacity: 1, x: 0 }}
                  exit={{ opacity: 0 }}
                  onClick={() => a.seek(e.time)}
                  className={s.event}
                >
                  <span
                    className={
                      e.kind === "target" ? s.eventHighlight : s.eventDot
                    }
                  />
                  <span>
                    <strong>{e.title}</strong>
                    <small>{e.detail}</small>
                  </span>
                  <time>{formatTime(e.time)}</time>
                </m.button>
              ))}
            </AnimatePresence>
          ) : (
            <div className={s.empty}>
              <Search size={20} />
              <strong>No events yet</strong>
              <p>Play forward or choose another filter.</p>
              <button onClick={() => setFilter("all")}>Show all events</button>
            </div>
          )}
        </div>
      </section>
    </aside>
  );
}
function Telemetry({ scenario }: { scenario: Scenario }) {
  const a = useAeris((v) => v);
  const [channel, setChannel] = useState<"coverage" | "battery">("coverage");
  const frames = recordings[scenario.id];
  const current = frames[Math.floor(a.time)];
  const points = frames
    .filter((_, i) => i % 3 === 0)
    .map((f) => `${40 + (f.time / 240) * 580},${125 - f[channel]}`)
    .join(" ");
  const id = useId().replaceAll(":", "");
  return (
    <section className={s.telemetry}>
      <div className={s.panelHeader}>
        <div>
          <h2>Mission telemetry</h2>
          <p>How does the mission evolve?</p>
        </div>
        <div className={s.segmented} aria-label="Telemetry channel">
          <button
            aria-pressed={channel === "coverage"}
            onClick={() => setChannel("coverage")}
          >
            Coverage
          </button>
          <button
            aria-pressed={channel === "battery"}
            onClick={() => setChannel("battery")}
          >
            Battery
          </button>
        </div>
      </div>
      <div className={s.chartReadout}>
        <span className={s.tinyDot} />
        {channel === "coverage" ? "Explored area" : "Remaining battery"}
        <strong>{current[channel].toFixed(1)}%</strong>
        <span>at {formatTime(a.time)}</span>
      </div>
      <svg
        viewBox="0 0 650 160"
        role="img"
        aria-label={`${channel} over 4 minutes. At ${formatTime(a.time)}: ${current[channel].toFixed(1)} percent. Change time using the mission timeline.`}
      >
        <defs>
          <clipPath id={id}>
            <rect x="0" y="0" width={40 + (a.time / 240) * 580} height="160" />
          </clipPath>
        </defs>
        {[0, 50, 100].map((v) => (
          <g key={v}>
            <line
              x1="40"
              x2="620"
              y1={125 - v}
              y2={125 - v}
              stroke="var(--line)"
              strokeDasharray="3 4"
            />
            <text x="0" y={129 - v} className={s.chartText}>
              {v}%
            </text>
          </g>
        ))}
        <polyline
          points={points}
          stroke="var(--line-strong)"
          strokeWidth="2"
          fill="none"
        />
        <g clipPath={`url(#${id})`}>
          <polygon
            points={`40,125 ${points} 620,125`}
            fill="var(--scan)"
            opacity=".22"
          />
          <m.polyline
            initial={{ pathLength: 0 }}
            animate={{ pathLength: 1 }}
            transition={{ duration: 0.6 }}
            points={points}
            stroke="var(--green)"
            strokeWidth="2.5"
            fill="none"
          />
        </g>
        <line
          x1={40 + (a.time / 240) * 580}
          x2={40 + (a.time / 240) * 580}
          y1="12"
          y2="125"
          stroke="var(--ink)"
          strokeWidth="1"
          strokeDasharray="3 3"
        />
        <circle
          cx={40 + (a.time / 240) * 580}
          cy={125 - current[channel]}
          r="4"
          fill="var(--green)"
          stroke="var(--surface)"
          strokeWidth="2"
        />
        {[0, 60, 120, 180, 240].map((v) => (
          <text
            key={v}
            x={40 + (v / 240) * 580}
            y="151"
            textAnchor="middle"
            className={s.chartText}
          >
            {formatTime(v)}
          </text>
        ))}
      </svg>
      <div className={s.chartFoot}>
        <span>
          <span className={s.tinyDot} />
          Synthetic model
        </span>
        <span>Drag the timeline above to inspect</span>
      </div>
    </section>
  );
}
function Analytics({ scenario }: { scenario: Scenario }) {
  const time = useAeris((v) => v.time);
  return (
    <div className={s.analytics}>
      <div className={s.mapPanel}>
        <Telemetry scenario={scenario} />
        <Timeline scenario={scenario} />
      </div>
      <section className={s.compare}>
        <div className={s.panelHeader}>
          <div>
            <h2>Worlds, side by side.</h2>
            <p>
              Explored area at {formatTime(time)} · identical replay route
              budget
            </p>
          </div>
          <FlaskConical size={20} />
        </div>
        {scenarios.map((v) => {
          const f = recordings[v.id][Math.floor(time)];
          return (
            <div className={s.comparisonRow} key={v.id}>
              <span>
                {v.name}
                <small>
                  Seed {v.seed} · {v.family}
                </small>
              </span>
              <div className={s.barTrack}>
                <m.div
                  initial={{ width: 0 }}
                  animate={{ width: `${f.coverage}%` }}
                  transition={{ duration: 0.6 }}
                />
              </div>
              <strong>{f.coverage.toFixed(1)}%</strong>
            </div>
          );
        })}
        <p className={s.compareNote}>
          An interface example, not a benchmark. Differences come from seeded
          obstacle geometry. No algorithm-performance claims are made.
        </p>
      </section>
      <div className={s.method}>
        <ShieldCheck size={23} />
        <div>
          <h2>Evidence has a provenance.</h2>
          <p>
            These values are generated in your browser from seed 303. Real AERIS
            evaluation distinguishes FastSim (Tier F) from high-fidelity
            PX4/Gazebo (Tier H). This showcase is neither.
          </p>
        </div>
      </div>
    </div>
  );
}
function Library({ onSelect }: { onSelect: (id: string) => void }) {
  const [query, setQuery] = useState("");
  const filtered = scenarios.filter((v) =>
    `${v.name} ${v.family}`.toLowerCase().includes(query.toLowerCase()),
  );
  return (
    <section className={s.library}>
      <div className={s.libraryHeading}>
        <h2>Find your next perspective.</h2>
        <label className={s.searchField}>
          <Search size={17} />
          <input
            placeholder="Search environments…"
            aria-label="Search environments"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
          />
        </label>
      </div>
      {filtered.length ? (
        <div className={s.worldGrid}>
          {filtered.map((v, i) => (
            <m.button
              layout
              className={s.worldCard}
              key={v.id}
              onClick={() => onSelect(v.id)}
              whileHover={{ y: -4 }}
            >
              <div className={s.worldPreview}>
                <svg viewBox="0 0 300 180" aria-hidden="true">
                  <rect
                    x="18"
                    y="18"
                    width="264"
                    height="144"
                    rx="2"
                    fill="none"
                    stroke="var(--wall)"
                    strokeWidth="2"
                  />
                  {v.obstacles.map((o, j) => (
                    <rect
                      key={j}
                      x={o.x / 2.5}
                      y={o.y / 2.5}
                      width={o.w / 2.5}
                      height={o.h / 2.5}
                      fill="var(--obstacle)"
                      stroke="var(--obstacle-line)"
                    />
                  ))}
                  <polyline
                    points={v.route
                      .map((p) => `${p.x / 2.5},${p.y / 2.5}`)
                      .join(" ")}
                    fill="none"
                    stroke="var(--green)"
                    strokeWidth="2"
                    strokeDasharray="3 4"
                  />
                </svg>
                <span>0{i + 1} / SEEDED WORLD</span>
              </div>
              <div className={s.worldInfo}>
                <span className={s.eyebrow}>{v.family}</span>
                <h3>
                  {v.name}
                  <ArrowUpRight size={19} />
                </h3>
                <p>240-second replay · 2 inspection points</p>
                <div>
                  <span>Seed {v.seed}</span>
                  <span>
                    Explore mission <ArrowRight size={14} />
                  </span>
                </div>
              </div>
            </m.button>
          ))}
        </div>
      ) : (
        <div className={s.empty}>
          <Search size={25} />
          <h3>No worlds found</h3>
          <p>Try “office”, “warehouse” or “rubble”.</p>
          <Button onClick={() => setQuery("")}>Clear search</Button>
        </div>
      )}
    </section>
  );
}
