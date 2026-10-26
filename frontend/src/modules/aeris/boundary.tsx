"use client";
import { Component, type ReactNode } from "react";
import { Radio } from "lucide-react";
import { Button } from "./ui/button";
import s from "./showcase.module.css";
/** Portable render recovery. No transport or backend side effects. */
export class ShowcaseBoundary extends Component<
  { children: ReactNode },
  { failed: boolean }
> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    if (!this.state.failed) return this.props.children;
    return (
      <div className={s.root}>
        <section role="alert" className={s.disconnected}>
          <Radio size={28} />
          <h2>The mission view couldn’t load</h2>
          <p>
            Your browser session can be reset safely. No vehicle is connected
            and no research data will be changed.
          </p>
          <Button onClick={() => this.setState({ failed: false })}>
            Reset demo
          </Button>
        </section>
      </div>
    );
  }
}
