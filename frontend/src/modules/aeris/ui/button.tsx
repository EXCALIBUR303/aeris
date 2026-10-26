import { Slot } from "@radix-ui/react-slot";
import { cva, type VariantProps } from "class-variance-authority";
import { clsx } from "clsx";
import styles from "../showcase.module.css";
// Locally owned shadcn Button pattern; module-scoped variants instead of global utility classes.
const variants = cva(styles.button, {
  variants: {
    variant: {
      default: styles.primary,
      outline: styles.outline,
      ghost: styles.ghost,
    },
    size: { default: "", icon: styles.iconButton },
  },
  defaultVariants: { variant: "outline", size: "default" },
});
export function Button({
  className,
  variant,
  size,
  asChild = false,
  ...props
}: React.ComponentProps<"button"> &
  VariantProps<typeof variants> & { asChild?: boolean }) {
  const Comp = asChild ? Slot : "button";
  return (
    <Comp className={clsx(variants({ variant, size }), className)} {...props} />
  );
}
