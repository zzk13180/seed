import { fileURLToPath } from 'node:url'
import { dirname, resolve } from 'node:path'
import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'

import { vitePreprocess } from '@sveltejs/vite-plugin-svelte'

const __filename = fileURLToPath(import.meta.url)
const __dirname = dirname(__filename)

export default defineConfig({
  build: {
    lib: {
      entry: './src/index.ts',
      name: 'custom-chat',
      fileName: 'index',
      formats: ['es'],
    },
    rollupOptions: {
      external: [],
      output: {
        inlineDynamicImports: true,
      },
    },
  },
  resolve: {
    alias: {
      $lib: resolve(__dirname, './src/lib'),
      '$lib/*': resolve(__dirname, './src/lib/*'),
    },
  },
  plugins: [
    svelte({
      configFile: false,
      preprocess: vitePreprocess(),
      compilerOptions: {
        customElement: true,
      },
    }),
    {
      name: 'exit-on-build-end',
      apply: 'build',
      closeBundle() {
        console.log('Build completed. Exiting process...')
        process.exit(0)
      },
    },
  ],
})
