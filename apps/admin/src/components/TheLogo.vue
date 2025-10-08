<template>
  <transition mode="out-in">
    <router-link key="expand" to="/" class="the-logo">
      <div :style="logoStyle"></div>
      <h1>{{ title }}</h1>
    </router-link>
  </transition>
</template>

<script setup lang="ts">
  const logoUrl = '/favicon/default.svg'
  const title = '@seed/admin'

  /**
   * 使用 CSS Mask 技术实现图标
   * 原理：以 SVG 为遮罩，通过 background-color 控制颜色
   */
  const logoStyle = computed(() => ({
    width: '24px',
    height: '24px',
    mask: `url(${logoUrl}) no-repeat center / contain`,
    backgroundColor: 'var(--el-text-color-primary)',
  }))
</script>

<style lang="scss" scoped>
  .the-logo {
    height: 100%;
    display: flex;
    justify-content: flex-start;
    align-items: center;
    gap: 4px;

    h1 {
      font-weight: 600;
      font-size: 16px;
      height: 100%;
      display: flex;
      align-items: center;
      color: var(--el-text-color-primary);
    }
  }
</style>
