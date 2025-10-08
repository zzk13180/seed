import type { DepOptimizationConfig } from 'vite'

/**
 * Vite 依赖预构建优化配置
 *
 * include: 强制预构建的依赖（加快启动速度）
 * exclude: 排除预构建的依赖（用于需要 ESM 的包）
 */
export const optimizeDeps: DepOptimizationConfig = {
  include: [
    // Vue 生态
    'vue',
    'vue-router',
    'pinia',
    // Element Plus（按需导入时仍需预构建核心）
    'element-plus/es',
    // 工具库
    'lodash-es',
    '@vueuse/core',
    // 可视化
    'echarts',
  ],
  exclude: [
    // 排除虚拟模块
    'virtual:svg-icons-register',
  ],
}
