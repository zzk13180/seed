<template>
  <div class="layout-sidebar-container">
    <el-menu
      :default-active="activeMenu"
      :collapse="true"
      mode="vertical"
      :popper-offset="12"
      popper-class="layout-sidebar-popper-menu"
      router
    >
      <el-scrollbar>
        <LayoutSidebarItem
          v-for="(r, index) in sidebarRouters"
          :key="r.path + index"
          :menu-item="r"
          :base-path="r.path"
        />
      </el-scrollbar>
    </el-menu>
  </div>
</template>

<script setup lang="ts">
  import { ref, computed } from 'vue'
  import { useRoute, useRouter } from 'vue-router'
  import LayoutSidebarItem from './LayoutSidebarItem.vue'

  const route = useRoute()
  const router = useRouter()

  const sidebarRouters = ref<any>(router.getRoutes())

  const activeMenu = computed<string>(() => {
    const { meta, path } = route
    // if set path, the sidebar will highlight the path you set
    if (typeof meta.activeMenu === 'string') {
      return meta.activeMenu
    }
    return typeof path === 'string' ? path : ''
  })
</script>

<style lang="scss" scoped>
  .layout-sidebar-container {
    width: var(--vv-sidebar-width);
    background-color: var(--el-bg-color);
    height: calc(100% - var(--vv-navbar-height));
    position: fixed;
    top: var(--vv-navbar-height);
    left: 0;
    z-index: 99;
    overflow: hidden;
    box-shadow:
      0 2px 4px -1px rgb(0 0 0 / 20%),
      0 4px 5px 0 rgb(0 0 0 / 14%),
      0 1px 10px 0 rgb(0 0 0 / 12%);

    :deep(.el-menu--vertical) {
      width: 100%; // 覆盖 .el-menu--collapse
      height: 100%;
      padding: 8px 0 8px 8px;
    }
  }
</style>
<style lang="scss">
  .el-popper.layout-sidebar-popper-menu {
    .el-popper__arrow {
      z-index: 2048;
    }
  }

  .el-menu--vertical.layout-sidebar-popper-menu {
    border-radius: 4px;

    .el-menu {
      min-width: var(--vv-sidebar-width);
      background-color: var(--el-bg-color);
      padding: 8px 0 8px 8px;
      border-radius: 4px;
      box-shadow: -4px 0 16px 0 #001c4d1f;
    }
  }
</style>
