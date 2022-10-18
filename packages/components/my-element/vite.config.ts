import { defineConfig } from 'vite'
import { resolve } from 'path'

export default defineConfig({
  base: '/my-element/',
  build: {
    lib: {
      entry: './index.ts',
      formats: ['es'],
    },
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
      },
    },
  },
})
