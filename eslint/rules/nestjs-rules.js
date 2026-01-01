/**
 * NestJS 应用专用规则
 * NestJS 使用装饰器和依赖注入，需要放宽部分规则
 */
export const nestjsRules = {
  '@typescript-eslint/parameter-properties': 'off', // 允许构造函数参数属性
  '@typescript-eslint/no-base-to-string': 'off', // 允许对象隐式转字符串
  '@typescript-eslint/require-await': 'off', // 允许无 await 的 async 函数
  'unicorn/prefer-top-level-await': 'off', // NestJS 入口不使用顶层 await
  'unicorn/no-process-exit': 'off', // 允许 process.exit()
  'unicorn/prefer-module': 'off', // 允许 CommonJS 语法
}
