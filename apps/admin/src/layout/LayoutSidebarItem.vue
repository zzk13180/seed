<template>
  <div v-if="menuItem.meta && !menuItem.meta.hideMenu" class="layout-menu-item">
    <!-- 单个子菜单项的情况 -->
    <template
      v-if="
        hasSingleVisibleChild(menuItem, menuItem.children) &&
        (!singleChild.children || singleChild.noVisibleChildren) &&
        !menuItem.alwaysShow
      "
    >
      <router-link v-if="singleChild.meta" :to="getFullPath(singleChild.path, singleChild.query)">
        <el-menu-item :index="getFullPath(singleChild.path).toString()">
          <svg-icon :name="singleChild.meta?.icon || menuItem.meta?.icon || ''" />
          <span :title="getTitleTooltip(singleChild.meta?.title)">{{
            singleChild.meta?.title
          }}</span>
        </el-menu-item>
      </router-link>
    </template>

    <!-- 多个子菜单项的情况 -->
    <el-sub-menu v-else ref="subMenu" :index="getFullPath(menuItem.path).toString()" teleported>
      <template v-if="menuItem.meta" #title>
        <svg-icon :name="menuItem.meta?.icon || ''" />
        <span :title="getTitleTooltip(menuItem.meta?.title)">
          {{ menuItem.meta?.title }}
        </span>
      </template>

      <LayoutSidebarItem
        v-for="(child, index) in menuItem.children"
        :key="child.path + index"
        :is-nest="true"
        :menu-item="child"
        :base-path="getFullPath(child.path).toString()"
      />
    </el-sub-menu>
  </div>
</template>

<script setup lang="ts">
  import { SidebarRouteItem } from './layout.types'

  interface Props {
    menuItem: SidebarRouteItem
    basePath?: string
  }

  const props = withDefaults(defineProps<Props>(), {
    basePath: '',
  })

  // 单个子菜单项的引用
  const singleChild = ref<SidebarRouteItem>({} as SidebarRouteItem)

  /**
   * 判断是否只有一个需要显示的子菜单项
   * @param parent 父菜单项配置
   * @param children 子菜单项配置数组
   * @returns 是否只有一个可显示的子菜单项
   */
  const hasSingleVisibleChild = (
    parent: SidebarRouteItem,
    children?: SidebarRouteItem[],
  ): boolean => {
    if (!children?.length) {
      // 当没有子菜单项时，显示父菜单项
      singleChild.value = {
        ...parent,
        path: '',
        noVisibleChildren: true,
      }
      return true
    }

    // 过滤出所有未隐藏的子菜单项
    const visibleChildren = children.filter(item => !item.hidden)

    // 当只有一个子菜单项时，显示该子菜单项
    if (visibleChildren.length === 1) {
      singleChild.value = visibleChildren[0]
      return true
    }

    // 当没有可显示的子菜单项时，显示父菜单项
    if (visibleChildren.length === 0) {
      singleChild.value = {
        ...parent,
        path: '',
        noVisibleChildren: true,
      }
      return true
    }

    return false
  }

  /**
   * 解析完整路径
   * @param routePath 路由路径
   * @param routeQuery 路由查询参数（JSON字符串）
   */

  const getFullPath = (routePath: string, routeQuery?: string): string | Record<string, any> => {
    // 拼接路径，避免多余的斜杠
    let path = props.basePath
      ? `${props.basePath.replace(/\/$/, '')}/${routePath.replace(/^\//, '')}`
      : routePath

    // 移除路径末尾的斜杠
    path = path.replace(/\/$/, '')

    // 处理带查询参数的路由
    if (routeQuery) {
      try {
        const query = JSON.parse(routeQuery)
        return { path, query }
      } catch (error) {
        console.error('无效的路由查询参数格式:', routeQuery, error)
      }
    }

    return path
  }

  /**
   * 获取标题提示文本
   * 只在标题过长时返回标题用于tooltip显示
   */
  const getTitleTooltip = (title?: string): string => {
    return title && title.length > 5 ? title : ''
  }
</script>

<style lang="scss">
  .layout-menu-item {
    margin-bottom: 2px;
    padding-right: 8px;

    .el-menu-item,
    .el-sub-menu__title {
      border-radius: 4px;

      span {
        padding-left: 8px;
      }
    }

    .el-menu-item:hover,
    .el-sub-menu__title:hover {
      background-color: var(--el-bg-color-page);
    }

    .el-sub-menu.is-active,
    .el-menu-item.is-active {
      span {
        font-weight: 600;
      }
    }
  }
</style>
