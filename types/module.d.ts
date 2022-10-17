/// <reference types="vite/client" />
/// <reference types="@idux/cdk/types" />
/// <reference types="@idux/components/types" />
/// <reference types="@idux/pro/types" />
/// <reference types="astro/client" />

declare module '*.vue' {
  import { DefineComponent } from 'vue'
  const Component: DefineComponent<{}, {}, any>
  export default Component
}
