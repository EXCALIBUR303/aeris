import type { Metadata } from "next";
import "./globals.css";
export const metadata: Metadata = {
  title: "AERIS — Autonomy, observed.",
  description:
    "Explore an interactive, reproducible drone mission. A portfolio demonstration of the AERIS autonomy research platform.",
};
export default function Layout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body style={{ margin: 0 }}>{children}</body>
    </html>
  );
}
