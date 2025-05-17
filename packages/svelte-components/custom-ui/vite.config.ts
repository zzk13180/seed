import { sveltekit } from '@sveltejs/kit/vite'
import { defineConfig } from 'vite'

export default defineConfig({
  server: {
    host: '0.0.0.0',
    port: 8090,
    open: true,
    proxy: {
      '/ws': {
        target: 'http://localhost:3000',
        ws: true,
      },
      '/api': {
        target: 'http://localhost:3000',
      },
      '/health': {
        target: 'http://localhost:3000',
      },
      '/ollama': {
        target: 'http://localhost:3000',
      },
    },
  },
  plugins: [sveltekit()],
})
