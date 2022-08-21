import { defineConfig } from 'vite-plugin-windicss'

export default defineConfig({
  theme: {
    screens: {
      sm: '600px',
      md: '960px',
      lg: '1280px',
      xl: '1720px',
    },
  },
})
