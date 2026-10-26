import { AerisShowcase } from "@/src/modules/aeris";
export default function Page() {
  return <AerisShowcase demo={process.env.NEXT_PUBLIC_DEMO_MODE !== "false"} />;
}
