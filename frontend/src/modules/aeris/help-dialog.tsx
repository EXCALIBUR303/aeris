"use client";
import { Modal } from "./ui/dialog";
import s from "./showcase.module.css";
export default function HelpDialog({
  open,
  onOpenChange,
  theme,
}: {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  theme: string;
}) {
  return (
    <Modal
      open={open}
      onOpenChange={onOpenChange}
      title="A little context. A lot to explore."
      description="AERIS is an autonomous drone research platform. This standalone showcase explains a mission without connecting to a vehicle."
      theme={theme}
    >
      <div className={s.guide}>
        <h3>Try your first mission</h3>
        <ol>
          <li>Choose a world in the sidebar or map menu.</li>
          <li>
            Play or scrub the replay. The map, metrics and events share one
            clock.
          </li>
          <li>
            Select an inspection point to jump to its event; compare worlds in
            Analytics.
          </li>
        </ol>
        <h3>Honest by design</h3>
        <p>
          All paths, telemetry and inspection annotations are seeded synthetic
          examples. Coverage counts visited free grid cells within an
          illustrative 8.3 m sensing radius. Battery is a simple distance-based
          model. This is neither a FastSim evaluation nor evidence of target
          detection or autonomous performance.
        </p>
        <div className={s.shortcuts}>
          <span>
            Play / pause <kbd>Space</kbd>
          </span>
          <span>
            Seek 10 seconds <kbd>← / →</kbd>
          </span>
          <span>
            Quick actions <kbd>⌘ / Ctrl K</kbd>
          </span>
          <span>
            Open this guide <kbd>?</kbd>
          </span>
        </div>
      </div>
    </Modal>
  );
}
