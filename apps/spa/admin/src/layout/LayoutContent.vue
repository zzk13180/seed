<template>
  <div class="layout-content-container">
    <section class="layout-content-wrapper">
      <router-view v-slot="{ Component, route }">
        <transition mode="out-in">
          <keep-alive v-if="!route.meta.noCache">
            <component :is="Component" :key="route.path" class="content-view" />
          </keep-alive>
          <component v-else :is="Component" :key="route.path" class="content-view" />
        </transition>
      </router-view>
    </section>
  </div>
</template>

<style lang="scss" scoped>
  .layout-content-container {
    height: 100%;
    width: 100%;
    padding-top: var(--vv-navbar-height);
    padding-left: var(--vv-sidebar-width);
    background-color: var(--el-bg-color-page);

    .layout-content-wrapper {
      height: 100%;
      width: 100%;
      position: relative;
      padding: 8px 16px 16px;

      .content-view {
        height: 100%;
        border-radius: var(--el-border-radius-small);
      }
    }
  }
</style>
