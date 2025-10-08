<template>
  <span class="svg-icon-wrapper" :style="iconStyle" role="img" aria-hidden="true">
    <svg class="svg-icon">
      <use :xlink:href="symbolId" />
    </svg>
  </span>
</template>

<script setup lang="ts">
  import { computed, type CSSProperties } from 'vue'

  interface SvgIconProps {
    /** 图标名称（对应 svg-icons 目录下的文件名） */
    name: string
    /** symbolId 前缀，默认 'icon' */
    prefix?: string
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
    prefix: 'icon',
    size: '1em',
  })

  const symbolId = computed(() => `#${props.prefix}-${props.name}`)

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
  }

  .svg-icon {
    width: 100%;
    height: 100%;
    fill: currentcolor;
    overflow: hidden;
  }
</style>
