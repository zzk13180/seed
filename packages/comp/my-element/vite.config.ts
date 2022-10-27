import { resolve } from 'path'
import { defineConfig } from 'vite'

export default defineConfig({
  base: '/my-element/',
  build: {
    lib: {
      entry: './index.ts',
      formats: ['es'],
    },
    rollupOptions: {
      external: /^lit/,
      input: {
        main: resolve(__dirname, 'index.html'),
      },
    },
  },
})
