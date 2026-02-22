<template>
  <span
    class="svg-icon-wrapper"
    :style="iconStyle"
    role="img"
    aria-hidden="true"
    v-html="svgContent"
  />
</template>

<script setup lang="ts">
  import { computed, type CSSProperties } from 'vue'

  /**
   * 使用 Vite 内置 import.meta.glob 加载所有 SVG 图标（零依赖，替代 vite-plugin-svg-icons）
   */
  const svgModules = import.meta.glob<string>('@/assets/svg-icons/**/*.svg', {
    query: '?raw',
    import: 'default',
    eager: true,
  })

  /** 构建 name → raw SVG 映射 */
  const svgMap = Object.fromEntries(
    Object.entries(svgModules).map(([path, raw]) => {
      const name = path.split('/').pop()!.replace('.svg', '')
      return [name, raw]
    }),
  )

  interface SvgIconProps {
    /** 图标名称（对应 svg-icons 目录下的文件名） */
    name: string
    /**
     * 图标颜色
     * - 支持任意 CSS 颜色值
     * - 默认继承父元素 color（currentColor）
     */
    color?: string
    /**
     * 图标尺寸
     * - 数字: 作为 px 值
     * - 字符串: 直接使用（如 '1.5em', '24px'）
     */
    size?: string | number
  }

  const props = withDefaults(defineProps<SvgIconProps>(), {
    size: '1em',
  })

  const svgContent = computed(() => svgMap[props.name] ?? '')

  const iconStyle = computed<CSSProperties>(() => {
    const size = typeof props.size === 'number' ? `${props.size}px` : props.size
    return {
      '--svg-icon-size': size,
      '--svg-icon-color': props.color || 'currentColor',
    } as CSSProperties
  })
</script>

<style scoped lang="scss">
  .svg-icon-wrapper {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    width: var(--svg-icon-size, 1em);
    height: var(--svg-icon-size, 1em);
    color: var(--svg-icon-color, currentColor);
    font-size: inherit;
    line-height: 1;

    :deep(svg) {
      width: 100%;
      height: 100%;
      fill: currentcolor;
    }
  }
</style>
