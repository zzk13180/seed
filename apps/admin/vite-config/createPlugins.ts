import path from 'path'
import vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import tailwindcss from '@tailwindcss/vite'
import { vitePluginCompression } from './vite-plugin-compression'
import { vitePluginSvgIcons } from './vite-plugin-svg-icons'
import { componentsAutoImportPlugin } from './auto-import-components'
import { VueAutoImportPlugin } from './auto-import-vue'
import type { Plugin } from 'vite'

export const createPlugins = (viteEnv: Record<string, any>, isBuild = false): Plugin[] => {
  const vitePlugins: Plugin[] = [
    vue({
      template: {
        compilerOptions: {
          isCustomElement: (tag: string) => tag.startsWith('custom-'),
        },
      },
    }),
    VueJsx(),
    vitePluginSvgIcons(path, isBuild),
    componentsAutoImportPlugin(path),
    VueAutoImportPlugin(path),
    ...tailwindcss(),
    ...vitePluginCompression(viteEnv),
  ]
  return vitePlugins
}
