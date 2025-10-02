import path from 'node:path'
import vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import { vitePluginCompression } from './compression'
import { vitePluginSvgIcons } from './svg-icon'
import type { Plugin } from 'vite'

export const createPlugins = (viteEnv: any, isBuild = false): Plugin[] => {
  const vitePlugins: Plugin[] = []
  vitePlugins.push(vue())
  vitePlugins.push(VueJsx())
  // @ts-ignore
  vitePlugins.push(vitePluginCompression(viteEnv))
  vitePlugins.push(vitePluginSvgIcons(path, isBuild))
  return vitePlugins
}
