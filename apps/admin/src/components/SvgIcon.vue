<template>
  <el-icon :color="color" :size="size">
    <svg aria-hidden="true" class="svg-icon" role="img">
      <use :xlink:href="symbolId" />
    </svg>
  </el-icon>
</template>

<script setup lang="ts">
  interface SvgIconProps {
    name: string
    prefix?: string
    color?: string
    size?: string | number
  }

  const props = withDefaults(defineProps<SvgIconProps>(), {
    prefix: 'icon',
  })

  const symbolId = computed(() => `#${props.prefix}-${props.name}`)
</script>

<style scoped lang="scss">
  .svg-icon {
    width: 1em;
    height: 1em;
    overflow: hidden;
  }
</style>
