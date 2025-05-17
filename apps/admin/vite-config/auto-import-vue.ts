import AutoImport from 'unplugin-auto-import/vite'
import { CustomElementPlusResolver } from './auto-import-resolver'
import type Path from 'node:path'
import type { Plugin } from 'vite'

export const VueAutoImportPlugin = (path: typeof Path) => {
  return AutoImport({
    imports: ['vue'],
    vueTemplate: true,
    resolvers: [CustomElementPlusResolver()],
    dts: path.resolve(path.resolve(import.meta.dirname, '../'), 'auto-import-vue.d.ts'),
    exclude: [/node_modules/, /packages/, /\.git/],
  }) as Plugin
}
