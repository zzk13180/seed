import { defineConfig } from 'vitest/config'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import { viteConfig } from '@seed/viteconfig'

export default defineConfig(env => {
  const plugins = [VueJsx(), Vue()]
  return {
    ...viteConfig(env),
    publicDir: 'public',
    envDir: 'env',
    plugins,
    test: {
      watch: false,
      globals: true,
      environment: 'jsdom',
      transformMode: {
        web: [/\.[jt]sx$/],
      },
      coverage: {
        enabled: true,
        reporter: ['json', 'lcov', 'cobertura'],
        include: ['./src/*.{ts,tsx}'],
        exclude: [],
      },
      reporters: ['default'],
      include: ['./__tests__/*.spec.{ts,tsx}'],
    },
    build: {
      outDir: './dist/',
      emptyOutDir: true,
      sourcemap: false,
      lib: {
        entry: 'src/index.ts',
        name: 'vue-components',
      },
      rollupOptions: {
        external: ['vue'],
        output: {
          globals: {
            vue: 'Vue',
          },
        },
      },
    },
  }
})
