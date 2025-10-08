import seedPreset from '@seed/tailwind-config/preset'
import type { Config } from 'tailwindcss'

export default {
  presets: [seedPreset],
  content: [`${import.meta.dirname}/src/**/*.vue`],
  theme: {
    extend: {},
  },
  plugins: [],
} satisfies Config
