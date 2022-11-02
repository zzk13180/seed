import type { UserOptions } from 'vite-plugin-windicss'

export const windiConfig: UserOptions = {
  config: {
    theme: {
      screens: {
        sm: '600px',
        md: '960px',
        lg: '1280px',
        xl: '1720px',
      },
    },
  },
}
