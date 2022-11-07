<template>
  <template v-if="mode === 'link'">
    <a :class="classes">
      <slot></slot>
    </a>
  </template>
  <template v-else>
    <button :class="classes" :disabled="!!(disabled || loading)" :type="type!">
      <slot></slot>
    </button>
  </template>
</template>

<script lang="ts" setup>
  import { computed, normalizeClass, useSlots } from 'vue'

  const props = withDefaults(
    defineProps<{
      mode?: 'primary' | 'default' | 'dashed' | 'text' | 'link'
      type?: 'button' | 'submit' | 'reset'
      size?: 'lg' | 'xl' | 'md' | 'sm' | 'xs'
      shape?: 'circle' | 'round' | ''
      danger?: boolean
      ghost?: boolean
      disabled?: boolean
      loading?: boolean
      block?: boolean
      icon?: string
    }>(),
    {
      mode: 'primary',
      type: 'button',
      size: 'md',
      shape: '',
      icon: '',
    },
  )

  const slots = useSlots()

  const classes = computed(() => {
    const { block, danger, disabled, ghost, size, mode, loading, icon, shape } = props
    return normalizeClass({
      [prefixCls]: true,
      [`${prefixCls}-block`]: block,
      [`${prefixCls}-danger`]: danger,
      [`${prefixCls}-disabled`]: disabled || loading,
      [`${prefixCls}-ghost`]: ghost,
      [`${prefixCls}-loading`]: loading,
      [`${prefixCls}-icon-only`]: !slots.default && (icon || loading),
      [`${prefixCls}-${mode}`]: mode,
      [`${prefixCls}-${shape}`]: !!shape,
      [`${prefixCls}-${size}`]: true,
    })
  })
  const prefixCls = `seed-sample-button`
</script>

<style lang="less">
  @import url('../style/index.less');
</style>
