<template>
  <div class="navbar">
    <!-- 左侧区域：Logo和顶部导航 -->
    <div class="navbar-left">
      <TheLogo />
    </div>

    <!-- 右侧区域：功能模块 -->
    <div class="navbar-right">
      <div class="nav-actions">
        <el-tooltip content="全屏" effect="dark" placement="bottom">
          <svg-icon
            style="cursor: pointer"
            :name="isFullscreen ? 'exit-fullscreen' : 'fullscreen'"
            color="#fff"
            @click="toggle"
          />
        </el-tooltip>
      </div>

      <!-- 功能按钮组 -->
      <div class="nav-actions">
        <div class="avatar-container">
          <vv-dropdown @command="handleCommand">
            <div class="avatar-wrapper">
              <img :src="profile" draggable="false" />
            </div>
            <template #dropdown>
              <vv-dropdown-menu class="vv-dropdown-menu">
                <div>
                  <router-link to="/user/profile">
                    <vv-dropdown-item>个人中心</vv-dropdown-item>
                  </router-link>
                </div>
                <vv-dropdown-item command="logout">
                  <span>退出登录</span>
                </vv-dropdown-item>
              </vv-dropdown-menu>
            </template>
          </vv-dropdown>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
  import 'element-plus/es/components/base/style/css'
  import 'element-plus/es/components/button/style/css'
  import 'element-plus/es/components/button-group/style/css'
  import 'element-plus/es/components/popper/style/css'
  import 'element-plus/es/components/scrollbar/style/css'
  import 'element-plus/theme-chalk/el-dropdown.css'
  import { useRouter } from 'vue-router'
  import { useFullscreen } from '@vueuse/core'
  import TheLogo from '@/components/TheLogo.vue'
  import profile from '@/assets/images/profile.jpeg'
  import { AccessTokenUtil } from '@/utils/token.util'

  const router = useRouter()

  const logout = () => {
    console.log('logout')
    AccessTokenUtil.removeToken()
    router.push('/login')
  }
  const commandMap: { [key: string]: any } = {
    logout,
  }
  const handleCommand = (command: string) => {
    if (commandMap[command]) {
      commandMap[command]()
    }
  }

  const { isFullscreen, toggle } = useFullscreen()
</script>

<style lang="scss" scoped>
  .navbar {
    height: var(--vv-navbar-height);
    width: 100%;
    padding: 0 20px;
    background-color: var(--el-color-primary);
    position: relative;
    display: flex;
    justify-content: space-between;
    align-items: center;

    &-left {
      display: flex;
      justify-content: flex-start;
      align-items: center;
      height: 100%;
    }

    &-right {
      height: 100%;
      display: flex;
      align-items: center;
      gap: 16px;
    }

    .navbar-right {
      .nav-actions {
        display: flex;
        align-items: center;
        gap: 16px;
      }
    }

    .avatar-container {
      height: 100%;
      display: flex;
      align-items: center;

      .avatar-wrapper {
        cursor: pointer;
        width: 24px;
        height: 24px;
        border: 1px solid var(--vv-navbar-avatar-border);
        border-radius: 4px;

        img {
          user-select: none;
          width: 100%;
          height: 100%;
          border-radius: 4px;
        }
      }
    }

    /* 保留其他样式，但考虑将一些样式移动到子组件中 */
    :deep(.nav-top-menu.el-menu--horizontal) {
      position: absolute;
      left: var(--vv-sidebar-width);
      top: 0;
      height: 100%;
      display: flex;
      align-items: center;
      justify-content: flex-start;
      background-color: var(--el-color-primary);
      border: 0;

      > .el-menu-item {
        color: #fff !important;
        font-weight: 400;
        font-size: 14px;
        border: 0;
        padding: 0 16px;
        opacity: 0.8;
      }

      > .el-menu-item:hover {
        opacity: 1;
      }

      > .el-menu-item.is-active {
        font-weight: 600;
        opacity: 1;
      }

      .el-menu-item:not(.is-disabled):focus,
      .el-menu-item:not(.is-disabled):hover {
        background-color: transparent;
      }

      .el-sub-menu {
        > .el-sub-menu__title {
          color: #fff !important;
          font-weight: 400;
          font-size: 14px;
          border: 0;
          opacity: 0.8;
        }

        > .el-sub-menu__title:hover {
          opacity: 1;
        }

        > .el-sub-menu__title.is-active {
          font-weight: 600;
          opacity: 1;
        }

        .el-sub-menu__title:not(.is-disabled):focus,
        .el-sub-menu__title:not(.is-disabled):hover {
          background-color: transparent;
        }
      }
    }
  }
</style>
