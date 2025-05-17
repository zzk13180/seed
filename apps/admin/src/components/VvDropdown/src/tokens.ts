import type { ComputedRef, InjectionKey, Ref } from 'vue'

export type DropdownInjectionContext = {
  contentRef: Ref<HTMLElement | null>
  role: ComputedRef<string>
  triggerId: ComputedRef<string>
  isUsingKeyboard: Ref<boolean>
  onItemLeave: (e: PointerEvent) => void
  onItemEnter: (e: PointerEvent) => void
}

export const DROPDOWN_INJECTION_KEY: InjectionKey<DropdownInjectionContext> = Symbol('elDropdown')
