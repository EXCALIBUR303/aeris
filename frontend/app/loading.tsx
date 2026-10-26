import s from "@/src/modules/aeris/showcase.module.css";
export default function Loading() {
  return (
    <div className={s.root}>
      <div className={s.mapSkeleton} role="status" aria-label="Loading AERIS">
        <div />
        <div />
        <span>Preparing your research workspace…</span>
      </div>
    </div>
  );
}
