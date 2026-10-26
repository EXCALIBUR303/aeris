"use client";
import * as Dialog from "@radix-ui/react-dialog";
import { X } from "lucide-react";
import { Button } from "./button";
import s from "../showcase.module.css";
export function Modal({
  open,
  onOpenChange,
  title,
  description,
  theme,
  children,
}: {
  open: boolean;
  onOpenChange: (v: boolean) => void;
  title: string;
  description: string;
  theme: string;
  children: React.ReactNode;
}) {
  return (
    <Dialog.Root open={open} onOpenChange={onOpenChange}>
      <Dialog.Portal>
        <Dialog.Overlay className={s.overlay} />
        <Dialog.Content className={`${s.root} ${s.dialog}`} data-theme={theme}>
          <div className={s.dialogHeader}>
            <Dialog.Title>{title}</Dialog.Title>
            <Dialog.Close asChild>
              <Button size="icon" variant="ghost" aria-label="Close dialog">
                <X size={18} />
              </Button>
            </Dialog.Close>
          </div>
          <Dialog.Description className={s.muted}>
            {description}
          </Dialog.Description>
          {children}
        </Dialog.Content>
      </Dialog.Portal>
    </Dialog.Root>
  );
}
