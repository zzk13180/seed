declare module '@seed/tailwind-config/preset' {
  import type { Config } from 'tailwindcss'
  const preset: Partial<Config>
  export default preset
}

declare module '@seed/tailwind-config' {
  import type { Config } from 'tailwindcss'
  export const seedPreset: Partial<Config>
  export function withSeedPreset(config: Config): Promise<Config>
}
