# TODO

1. use-lockscreen 防止页面跳动 注释部分代码
2. packages/components/pagination/src/components/sizes.vue 修改样式和设计图一致
3. packages/components/table/src/table/defaults.ts 修改 stripe 默认值为 true
4. packages/components/message-box/src/index.vue 去除 el-message-box__container 内图标的显示
5. packages/components/table/src/table.vue 数据为空时的默认样式
6. packages/components/card/src/card.vue 增加 empty 插槽
7. 将 el-dropdown 组件 移入 src/components
8. 添加 @seed/element-plus-icons 包，修改默认 icon。
9. packages/components/menu/src/menu.ts sub-menu-more 添加 popperClass: props.popperClass
10. packages/components/menu/src/sub-menu.ts fallbackPlacements horizontal 移除 'left-start'
11. packages/components/menu/src/sub-menu.ts showArrow: false, 修改为 showArrow: true menu 弹出的 popper 显示箭头
12. packages/components/table/src/config.ts CaretRight 替换 ArrowRight
13. packages/components/table/src/table-column/defaults.ts showOverflowTooltip 默认值 undefined 修改为 true
14. packages/components/button/src/button.vue 增加 props.content 用于通过属性定义按钮内容
15. packages/components/message-box/src/index.vue packages/components/color-picker/src/color-picker.vue 交换确定和取消按钮的位置
16. packages/components/menu/src/menu.ts TODO
17. packages/element-plus/components/form/src/form.vue @submit.prevent
18. packages/element-plus/components/form/src/form-item.vue 移除 :for="labelFor"
