"use client";
import { createContext, useContext, useState } from "react";
import { createStore, useStore } from "zustand";
export type View = "mission" | "analysis" | "runs";
type State = {
  scenario: string;
  time: number;
  playing: boolean;
  speed: number;
  theme: "light" | "dark";
  view: View;
  route: boolean;
  scan: boolean;
  setScenario: (id: string) => void;
  seek: (t: number) => void;
  tick: () => void;
  toggle: () => void;
  setSpeed: (s: number) => void;
  setTheme: (s: "light" | "dark") => void;
  setView: (s: View) => void;
  setLayer: (s: "route" | "scan") => void;
};
export const makeStore = () =>
  createStore<State>((set) => ({
    scenario: "warehouse",
    time: 128,
    playing: false,
    speed: 1,
    theme: "light",
    view: "mission",
    route: true,
    scan: true,
    setScenario: (scenario) => set({ scenario, time: 0, playing: false }),
    seek: (time) =>
      set({ time: Math.max(0, Math.min(240, time)), playing: false }),
    tick: () =>
      set((s) => ({
        time: Math.min(240, s.time + 0.25 * s.speed),
        playing: s.time + 0.25 * s.speed < 240,
      })),
    toggle: () =>
      set((s) => ({ playing: !s.playing, time: s.time >= 240 ? 0 : s.time })),
    setSpeed: (speed) => set({ speed }),
    setTheme: (theme) => set({ theme }),
    setView: (view) => set({ view }),
    setLayer: (layer) => set((s) => ({ [layer]: !s[layer] })),
  }));
const Context = createContext<ReturnType<typeof makeStore> | null>(null);
export function StoreProvider({ children }: { children: React.ReactNode }) {
  const [store] = useState(makeStore);
  return <Context.Provider value={store}>{children}</Context.Provider>;
}
export function useAeris<T>(selector: (state: State) => T): T {
  const store = useContext(Context);
  if (!store) throw new Error("Aeris store provider missing");
  return useStore(store, selector);
}
