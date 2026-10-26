"use client";
import { Command } from "cmdk";
import {
  Activity,
  ArrowDownToLine,
  ArrowUpRight,
  Layers,
  Map,
  Play,
  Sun,
} from "lucide-react";
import type { View } from "./store";
import { scenarios } from "./data";
import { Modal } from "./ui/dialog";
import s from "./showcase.module.css";
export default function CommandPalette({
  open,
  onOpenChange,
  theme,
  demo,
  onView,
  onScenario,
  onToggle,
  onTheme,
  onExport,
}: {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  theme: string;
  demo: boolean;
  onView: (view: View) => void;
  onScenario: (id: string) => void;
  onToggle: () => void;
  onTheme: () => void;
  onExport: () => void;
}) {
  const tabs: { id: View; label: string; icon: typeof Map }[] = [
    { id: "mission", label: "Mission view", icon: Map },
    { id: "analysis", label: "Analytics", icon: Activity },
    { id: "runs", label: "Mission library", icon: Layers },
  ];
  return (
    <Modal
      open={open}
      onOpenChange={onOpenChange}
      title="Mission command center"
      description="Navigate the showcase. Nothing here controls a vehicle."
      theme={theme}
    >
      <Command className={s.command}>
        <Command.Input
          placeholder="Search views, worlds, actions…"
          aria-label="Search commands"
        />
        <Command.List>
          <Command.Empty>
            No matching actions. Try “warehouse” or “theme”.
          </Command.Empty>
          <Command.Group heading="Workspace">
            {tabs.map((t) => (
              <Command.Item
                key={t.id}
                onSelect={() => {
                  onView(t.id);
                  onOpenChange(false);
                }}
              >
                <t.icon size={16} />
                {t.label}
                <ArrowUpRight size={14} />
              </Command.Item>
            ))}
          </Command.Group>
          <Command.Group heading="Explore">
            {scenarios.map((v) => (
              <Command.Item
                key={v.id}
                onSelect={() => {
                  onScenario(v.id);
                  onView("mission");
                  onOpenChange(false);
                }}
              >
                <Map size={16} />
                {v.name}
              </Command.Item>
            ))}
          </Command.Group>
          <Command.Group heading="Actions">
            <Command.Item
              disabled={!demo}
              onSelect={() => {
                onToggle();
                onOpenChange(false);
              }}
            >
              <Play size={16} />
              Play or pause replay
            </Command.Item>
            <Command.Item
              onSelect={() => {
                onTheme();
                onOpenChange(false);
              }}
            >
              <Sun size={16} />
              Toggle theme
            </Command.Item>
            <Command.Item
              disabled={!demo}
              onSelect={() => {
                onExport();
                onOpenChange(false);
              }}
            >
              <ArrowDownToLine size={16} />
              Export snapshot
            </Command.Item>
          </Command.Group>
        </Command.List>
      </Command>
    </Modal>
  );
}
