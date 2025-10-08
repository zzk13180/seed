<template>
  <div class="the-layout-container">
    <LayoutSidebar v-if="!appStore.state.sidebarIsHidden" />
    <div
      :class="{ 'layout-sidebar-hide': appStore.state.sidebarIsHidden }"
      class="layout-content-container"
    >
      <div class="layout-fixed-header">
        <LayoutNavbar />
      </div>
      <LayoutContent />
    </div>
  </div>
</template>

<script setup lang="ts">
  import { useAppStore } from '@/stores/app.store'
  import LayoutSidebar from './LayoutSidebar.vue'
  import LayoutNavbar from './LayoutNavbar.vue'
  import LayoutContent from './LayoutContent.vue'

  const appStore = useAppStore()
</script>

<style lang="scss" scoped>
  .the-layout-container {
    position: relative;
    height: 100%;
    width: 100%;

    .layout-content-container {
      height: 100%;
      position: relative;
    }

    .layout-fixed-header {
      position: fixed;
      top: 0;
      right: 0;
      z-index: 99;
      width: 100%;
    }
  }
</style>
