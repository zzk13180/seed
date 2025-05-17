import type { Config } from 'tailwindcss'

export default {
  content: [`${import.meta.dirname}/src/**/*.vue`],
  theme: {
    extend: {},
  },
  plugins: [],
} satisfies Config
