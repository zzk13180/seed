import { defineConfig } from 'astro/config'
import preact from '@astrojs/preact'
import react from '@astrojs/react'

export default defineConfig({
  integrations: [
    // Enable Preact to support Preact JSX components.
    preact(),
    // Enable React for the Algolia search component.
    react(),
  ],
  site: 'https://zzk13180.github.io',
  base: '/seed',
  output: 'static',
  build: {
    format: 'file',
  },
})
