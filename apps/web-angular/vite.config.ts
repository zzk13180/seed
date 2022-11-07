import { defineConfig } from 'vite'
import angular from '@analogjs/vite-plugin-angular'
import { viteConfig } from '@seed/viteconfig'

export default defineConfig(env => {
  const plugins = [angular()]
  return {
    ...viteConfig(env),
    plugins,
    root: 'src',
    publicDir: 'assets',
    build: {
      outDir: './dist/',
      emptyOutDir: true,
      target: 'es2020',
    },
    resolve: {
      mainFields: ['module'],
    },
  }
})
