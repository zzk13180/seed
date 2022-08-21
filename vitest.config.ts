import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'

export default {
  plugins: [VueJsx(), Vue()],
  test: {
    environment: 'jsdom',
    include: ['**/*.test.ts'],
  },
}
