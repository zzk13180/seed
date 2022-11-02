/// <reference types="vite/client" />
/// <reference types="astro/client" />

declare module '*.vue' {
  import type { DefineComponent } from 'vue'

  const Component: DefineComponent<{}, {}, any>
  export default Component
}
