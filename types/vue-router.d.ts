export {}
declare module 'vue-router' {
  interface RouteMeta extends Record<string | number | symbol, unknown> {
    title?: string
    // 菜单排序
    orderNo?: number
    // 是否忽略权限控制
    ignoreAuth?: boolean
    // 是否忽略KeepAlive缓存
    ignoreKeepAlive?: boolean
    // MultipleTab 固定位置不可关闭和拖动
    affix?: boolean
    // 菜单图标
    icon?: string
    // 内嵌iframe的地址
    frameSrc?: string
    // 切换路由动画
    transitionName?: string
    // 是否显示面包屑导航
    hideBreadcrumb?: boolean
    // 是否隐藏所有子菜单
    hideChildrenInMenu?: boolean
    // 标记为单层菜单
    single?: boolean
    // 当前激活的菜单
    currentActiveMenu?: string
    // 不在 MultipleTab 显示
    hideTab?: boolean
    // 不在菜单中显示
    hideMenu?: boolean
    // 拼接path时是否使用parentPath
    hidePathForChildren?: boolean
    // 是否基本的静态路由( resetRouter 重置路由时不会移除)
    isBasic?: boolean = false
  }
}
