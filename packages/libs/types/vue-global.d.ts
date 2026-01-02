/* eslint-disable sonarjs/unused-import */
/* eslint-disable @typescript-eslint/consistent-type-imports */
import type {
  App,
  Ref,
  ComputedRef,
  WatchEffect,
  WatchSource,
  WatchCallback,
  WatchOptions,
  WatchStopHandle,
  EffectScope,
  ComponentInternalInstance,
  VNode,
  Component,
  Directive,
  AsyncComponentLoader,
  DefineComponent,
  CustomRefFactory,
  ToRefs,
  UnwrapRef,
  ShallowRef,
  DeepReadonly,
  UnwrapNestedRefs,
} from 'vue'

declare global {
  // ===== 应用 API =====
  const createApp: (typeof import('vue'))['createApp']

  // ===== 基础响应式 API =====
  const ref: (typeof import('vue'))['ref']
  const reactive: (typeof import('vue'))['reactive']
  const readonly: (typeof import('vue'))['readonly']
  const shallowRef: (typeof import('vue'))['shallowRef']
  const shallowReactive: (typeof import('vue'))['shallowReactive']
  const shallowReadonly: (typeof import('vue'))['shallowReadonly']

  // ===== 响应式工具 =====
  const isRef: (typeof import('vue'))['isRef']
  const isReactive: (typeof import('vue'))['isReactive']
  const isReadonly: (typeof import('vue'))['isReadonly']
  const isProxy: (typeof import('vue'))['isProxy']
  const unref: (typeof import('vue'))['unref']
  const toRef: (typeof import('vue'))['toRef']
  const toRefs: (typeof import('vue'))['toRefs']
  const toRaw: (typeof import('vue'))['toRaw']
  const markRaw: (typeof import('vue'))['markRaw']

  // ===== 计算属性与侦听器 =====
  const computed: (typeof import('vue'))['computed']
  const watch: (typeof import('vue'))['watch']
  const watchEffect: (typeof import('vue'))['watchEffect']
  const watchPostEffect: (typeof import('vue'))['watchPostEffect']
  const watchSyncEffect: (typeof import('vue'))['watchSyncEffect']

  // ===== 生命周期钩子 =====
  const onBeforeMount: (typeof import('vue'))['onBeforeMount']
  const onMounted: (typeof import('vue'))['onMounted']
  const onBeforeUpdate: (typeof import('vue'))['onBeforeUpdate']
  const onUpdated: (typeof import('vue'))['onUpdated']
  const onBeforeUnmount: (typeof import('vue'))['onBeforeUnmount']
  const onUnmounted: (typeof import('vue'))['onUnmounted']
  const onActivated: (typeof import('vue'))['onActivated']
  const onDeactivated: (typeof import('vue'))['onDeactivated']
  const onErrorCaptured: (typeof import('vue'))['onErrorCaptured']
  const onRenderTracked: (typeof import('vue'))['onRenderTracked']
  const onRenderTriggered: (typeof import('vue'))['onRenderTriggered']
  const onServerPrefetch: (typeof import('vue'))['onServerPrefetch']

  // ===== 依赖注入 =====
  const provide: (typeof import('vue'))['provide']
  const inject: (typeof import('vue'))['inject']

  // ===== 组件 =====
  const defineComponent: (typeof import('vue'))['defineComponent']
  const defineAsyncComponent: (typeof import('vue'))['defineAsyncComponent']

  // ===== 渲染函数 =====
  const h: (typeof import('vue'))['h']
  const resolveComponent: (typeof import('vue'))['resolveComponent']
  const resolveDirective: (typeof import('vue'))['resolveDirective']

  // ===== 高级 API =====
  const getCurrentInstance: (typeof import('vue'))['getCurrentInstance']
  const getCurrentScope: (typeof import('vue'))['getCurrentScope']
  const effectScope: (typeof import('vue'))['effectScope']
  const onScopeDispose: (typeof import('vue'))['onScopeDispose']
  const customRef: (typeof import('vue'))['customRef']
  const triggerRef: (typeof import('vue'))['triggerRef']
  const nextTick: (typeof import('vue'))['nextTick']

  // ===== 类型定义 =====
  type ComponentInternalInstance = import('vue')['ComponentInternalInstance']
  type EffectScope = import('vue')['EffectScope']
}

// function vueGlobals() {
//   return {
//     // 类型定义
//     ComponentInternalInstance: true, // Vue 组件内部实例的类型定义
//     // 示例: const instance: ComponentInternalInstance | null = getCurrentInstance()

//     EffectScope: true, // 副作用作用域的类型定义
//     // 示例: const scope: EffectScope = effectScope()

//     // 计算属性相关
//     computed: true, // 创建计算属性
//     // 示例: const doubledValue = computed(() => count.value * 2)

//     computedAsync: true, // 创建异步计算属性 (VueUse)
//     // 示例: const data = computedAsync(async () => await fetchData())

//     computedEager: true, // 创建急切计算属性，立即执行 (VueUse)
//     // 示例: const result = computedEager(() => expensiveComputation())

//     computedInject: true, // 创建注入式计算属性 (VueUse)
//     // 示例: const injectedValue = computedInject('key', () => defaultComputation())

//     computedWithControl: true, // 创建可控制的计算属性 (VueUse)
//     // 示例: const { state, trigger } = computedWithControl(() => count.value * 2)

//     controlledComputed: true, // 受控计算属性 (VueUse)
//     // 示例: const controlled = controlledComputed(() => count.value * 2)

//     controlledRef: true, // 创建受控的响应式引用 (VueUse)
//     // 示例: const { state, setState } = controlledRef(0)

//     // 应用创建
//     createApp: true, // 创建 Vue 应用实例
//     // 示例: const app = createApp(App)

//     // 工具函数创建
//     createEventHook: true, // 创建事件钩子 (VueUse)
//     // 示例: const { on, trigger } = createEventHook<string>()

//     createGlobalState: true, // 创建全局状态 (VueUse)
//     // 示例: const useGlobalState = createGlobalState(() => ref(0))

//     createInjectionState: true, // 创建注入状态 (VueUse)
//     // 示例: const [useProvideState, useInjectState] = createInjectionState(() => ref(0))

//     createReactiveFn: true, // 创建响应式函数 (VueUse)
//     // 示例: const reactiveAdd = createReactiveFn((a, b) => a + b)

//     createSharedComposable: true, // 创建共享组合式函数 (VueUse)
//     // 示例: const useSharedMouse = createSharedComposable(useMouse)

//     createUnrefFn: true, // 创建解引用函数 (VueUse)
//     // 示例: const add = createUnrefFn((a, b) => a + b)

//     // 引用相关
//     customRef: true, // 创建自定义 ref
//     // 示例: const myRef = customRef((track, trigger) => ({ get: () => { track(); return value }, set: (v) => { value = v; trigger() } }))

//     debouncedRef: true, // 创建防抖 ref (VueUse)
//     // 示例: const debouncedValue = debouncedRef(input, 500)

//     debouncedWatch: true, // 创建防抖监听器 (VueUse)
//     // 示例: debouncedWatch(source, callback, { debounce: 500 })

//     // 组件定义
//     defineAsyncComponent: true, // 定义异步组件
//     // 示例: const AsyncComp = defineAsyncComponent(() => import('./AsyncComponent.vue'))

//     defineComponent: true, // 定义组件
//     // 示例: const MyComponent = defineComponent({ name: 'MyComponent', setup() { return {} } })

//     // 计算属性别名
//     eagerComputed: true, // 急切计算属性的别名
//     // 示例: const result = eagerComputed(() => heavyComputation())

//     // 作用域管理
//     effectScope: true, // 创建副作用作用域
//     // 示例: const scope = effectScope(); scope.run(() => { watch(source, callback) })

//     extendRef: true, // 扩展 ref 功能 (VueUse)
//     // 示例: const extended = extendRef(ref(0), { increment() { this.value++ } })

//     // 实例获取
//     getCurrentInstance: true, // 获取当前组件实例
//     // 示例: const instance = getCurrentInstance()

//     getCurrentScope: true, // 获取当前副作用作用域
//     // 示例: const scope = getCurrentScope()

//     // 渲染函数
//     h: true, // 创建虚拟 DOM 节点
//     // 示例: h('div', { class: 'container' }, 'Hello World')

//     // 监听器
//     ignorableWatch: true, // 创建可忽略的监听器 (VueUse)
//     // 示例: const { stop, ignoreUpdates } = ignorableWatch(source, callback)

//     // 依赖注入
//     inject: true, // 注入依赖
//     // 示例: const theme = inject('theme', 'light')

//     // 类型判断
//     isDefined: true, // 判断值是否已定义 (VueUse)
//     // 示例: if (isDefined(value)) { /* value 不是 null 或 undefined */ }

//     isProxy: true, // 判断是否为代理对象
//     // 示例: if (isProxy(obj)) { /* obj 是响应式代理 */ }

//     isReactive: true, // 判断是否为响应式对象
//     // 示例: if (isReactive(obj)) { /* obj 是响应式的 */ }

//     isReadonly: true, // 判断是否为只读对象
//     // 示例: if (isReadonly(obj)) { /* obj 是只读的 */ }

//     isRef: true, // 判断是否为 ref 对象
//     // 示例: if (isRef(value)) { /* value 是 ref */ }

//     // 工具函数
//     makeDestructurable: true, // 使对象可解构 (VueUse)
//     // 示例: const obj = makeDestructurable({ x: ref(0), y: ref(0) }, [toRef(obj, 'x'), toRef(obj, 'y')])

//     markRaw: true, // 标记对象为原始对象，不进行响应式转换
//     // 示例: const rawObj = markRaw({ foo: 'bar' })

//     nextTick: true, // 等待下一个 DOM 更新周期
//     // 示例: await nextTick(); // DOM 已更新

//     // 生命周期钩子
//     onActivated: true, // keep-alive 组件激活时触发
//     // 示例: onActivated(() => { console.log('组件激活') })

//     onBeforeMount: true, // 组件挂载前触发
//     // 示例: onBeforeMount(() => { console.log('即将挂载') })

//     onBeforeUnmount: true, // 组件卸载前触发
//     // 示例: onBeforeUnmount(() => { clearInterval(timer) })

//     onBeforeUpdate: true, // 组件更新前触发
//     // 示例: onBeforeUpdate(() => { console.log('即将更新') })

//     onClickOutside: true, // 点击元素外部时触发 (VueUse)
//     // 示例: onClickOutside(target, () => { isOpen.value = false })

//     onDeactivated: true, // keep-alive 组件停用时触发
//     // 示例: onDeactivated(() => { console.log('组件停用') })

//     onErrorCaptured: true, // 捕获子组件错误
//     // 示例: onErrorCaptured((err, instance, info) => { console.error(err) })

//     onKeyStroke: true, // 监听键盘按键 (VueUse)
//     // 示例: onKeyStroke('Enter', () => { submitForm() })

//     onLongPress: true, // 监听长按事件 (VueUse)
//     // 示例: onLongPress(target, () => { showContextMenu() })

//     onMounted: true, // 组件挂载后触发
//     // 示例: onMounted(() => { fetchData() })

//     onRenderTracked: true, // 渲染过程中依赖被追踪时触发
//     // 示例: onRenderTracked((e) => { console.log('依赖追踪', e) })

//     onRenderTriggered: true, // 渲染被触发时调用
//     // 示例: onRenderTriggered((e) => { console.log('渲染触发', e) })

//     onScopeDispose: true, // 副作用作用域销毁时触发
//     // 示例: onScopeDispose(() => { cleanup() })

//     onServerPrefetch: true, // 服务端渲染时数据预取
//     // 示例: onServerPrefetch(async () => { await fetchInitialData() })

//     onStartTyping: true, // 开始输入时触发 (VueUse)
//     // 示例: onStartTyping(() => { showSuggestions() })

//     onUnmounted: true, // 组件卸载后触发
//     // 示例: onUnmounted(() => { removeEventListeners() })

//     onUpdated: true, // 组件更新后触发
//     // 示例: onUpdated(() => { updateThirdPartyLibrary() })

//     // 依赖提供
//     provide: true, // 提供依赖给子组件
//     // 示例: provide('theme', 'dark')

//     // 响应式转换
//     reactify: true, // 将普通函数转为响应式函数 (VueUse)
//     // 示例: const reactiveAdd = reactify((a, b) => a + b)

//     reactifyObject: true, // 将对象的方法转为响应式 (VueUse)
//     // 示例: const reactiveObj = reactifyObject(Math)

//     reactive: true, // 创建响应式对象
//     // 示例: const state = reactive({ count: 0, name: 'Vue' })

//     reactiveComputed: true, // 响应式计算属性 (VueUse)
//     // 示例: const computed = reactiveComputed(() => ({ doubled: count.value * 2 }))

//     reactiveOmit: true, // 响应式地忽略对象属性 (VueUse)
//     // 示例: const omitted = reactiveOmit(obj, 'password')

//     reactivePick: true, // 响应式地选择对象属性 (VueUse)
//     // 示例: const picked = reactivePick(obj, 'name', 'age')

//     readonly: true, // 创建只读代理
//     // 示例: const readonlyState = readonly(state)

//     // ref 相关
//     ref: true, // 创建响应式引用
//     // 示例: const count = ref(0)

//     refAutoReset: true, // 自动重置的 ref (VueUse)
//     // 示例: const counter = refAutoReset(0, 1000) // 1秒后自动重置为0

//     refDebounced: true, // 防抖 ref (VueUse)
//     // 示例: const debouncedInput = refDebounced(input, 300)

//     refDefault: true, // 带默认值的 ref (VueUse)
//     // 示例: const value = refDefault(source, 'default')

//     refThrottled: true, // 节流 ref (VueUse)
//     // 示例: const throttledValue = refThrottled(input, 1000)

//     refWithControl: true, // 可控制的 ref (VueUse)
//     // 示例: const { state, set, reset } = refWithControl(0)

//     // 组件解析
//     resolveComponent: true, // 解析注册的组件
//     // 示例: const MyComponent = resolveComponent('MyComponent')

//     resolveDirective: true, // 解析注册的指令
//     // 示例: const vFocus = resolveDirective('focus')

//     // ref 解析
//     resolveRef: true, // 解析 ref (VueUse)
//     // 示例: const resolved = resolveRef(maybeRef)

//     resolveUnref: true, // 解析并解引用 (VueUse)
//     // 示例: const value = resolveUnref(refOrValue)

//     // 浅层响应式
//     shallowReactive: true, // 创建浅层响应式对象
//     // 示例: const state = shallowReactive({ nested: { count: 0 } })

//     shallowReadonly: true, // 创建浅层只读对象
//     // 示例: const readonly = shallowReadonly(state)

//     shallowRef: true, // 创建浅层 ref
//     // 示例: const shallowState = shallowRef({ count: 0 })

//     // ref 同步
//     syncRef: true, // 同步两个 ref (VueUse)
//     // 示例: syncRef(left, right)

//     syncRefs: true, // 同步多个 ref (VueUse)
//     // 示例: syncRefs(source, [target1, target2])

//     templateRef: true, // 模板引用 (VueUse)
//     // 示例: const el = templateRef<HTMLElement>('elementRef')

//     throttledRef: true, // 节流 ref (VueUse)
//     // 示例: const throttled = throttledRef(source, 1000)

//     throttledWatch: true, // 节流监听器 (VueUse)
//     // 示例: throttledWatch(source, callback, { throttle: 1000 })

//     // 转换函数
//     toRaw: true, // 返回响应式对象的原始版本
//     // 示例: const raw = toRaw(reactiveObj)

//     toReactive: true, // 将 ref 转换为响应式对象 (VueUse)
//     // 示例: const reactive = toReactive(refObj)

//     toRef: true, // 将响应式对象的属性转为 ref
//     // 示例: const nameRef = toRef(state, 'name')

//     toRefs: true, // 将响应式对象转为普通对象，每个属性都是 ref
//     // 示例: const { name, age } = toRefs(state)

//     triggerRef: true, // 手动触发 ref 更新
//     // 示例: triggerRef(customRef)

//     // 解引用
//     unref: true, // 解引用，如果是 ref 则返回 .value，否则返回原值
//     // 示例: const value = unref(maybeRef)

//     // 监听器
//     watch: true, // 监听响应式数据变化
//     // 示例: watch(source, (newVal, oldVal) => { console.log(newVal, oldVal) })

//     watchArray: true, // 监听数组变化 (VueUse)
//     // 示例: watchArray(list, (newList, oldList, added, removed) => {})

//     watchAtMost: true, // 最多触发指定次数的监听器 (VueUse)
//     // 示例: watchAtMost(source, callback, { count: 3 })

//     watchDebounced: true, // 防抖监听器 (VueUse)
//     // 示例: watchDebounced(source, callback, { debounce: 500 })

//     watchEffect: true, // 立即执行副作用函数并监听依赖变化
//     // 示例: watchEffect(() => { console.log(count.value) })

//     watchIgnorable: true, // 可忽略的监听器 (VueUse)
//     // 示例: const { ignoreUpdates } = watchIgnorable(source, callback)

//     watchOnce: true, // 只触发一次的监听器 (VueUse)
//     // 示例: watchOnce(source, callback)

//     watchPausable: true, // 可暂停的监听器 (VueUse)
//     // 示例: const { pause, resume } = watchPausable(source, callback)

//     watchPostEffect: true, // 在组件更新后执行的副作用
//     // 示例: watchPostEffect(() => { updateDOM() })

//     watchSyncEffect: true, // 同步执行的副作用
//     // 示例: watchSyncEffect(() => { syncOperation() })

//     watchThrottled: true, // 节流监听器 (VueUse)
//     // 示例: watchThrottled(source, callback, { throttle: 1000 })

//     watchTriggerable: true, // 可手动触发的监听器 (VueUse)
//     // 示例: const { trigger } = watchTriggerable(source, callback)

//     watchWithFilter: true, // 带过滤器的监听器 (VueUse)
//     // 示例: watchWithFilter(source, callback, { eventFilter: debounceFilter(500) })

//     whenever: true, // 条件监听器 (VueUse)
//     // 示例: whenever(isReady, () => { startApplication() })
//   }
// }
