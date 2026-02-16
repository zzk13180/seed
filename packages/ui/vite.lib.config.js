import { resolve } from 'node:path'
import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'

import { vitePreprocess } from '@sveltejs/vite-plugin-svelte'

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
      $lib: resolve(import.meta.dirname, './src/lib'),
      '$lib/*': resolve(import.meta.dirname, './src/lib/*'),
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
        throw new Error('Build completed')
      },
    },
  ],
})
