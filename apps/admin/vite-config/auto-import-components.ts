import Components from 'unplugin-vue-components/vite'
import { CustomElementPlusResolver } from './auto-import-resolver'
import type Path from 'node:path'
import type { Plugin } from 'vite'

export const componentsAutoImportPlugin = (path: typeof Path) => {
  return Components({
    resolvers: [CustomElementPlusResolver()],
    dts: path.resolve(path.resolve(import.meta.dirname, '../'), 'auto-import-components.d.ts'),
    exclude: [/node_modules/, /packages/, /\.git/],
  }) as Plugin
}
