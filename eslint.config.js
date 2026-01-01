/**
 * ESLint 配置文件
 *
 * 本配置文件采用 ESLint Flat Config 格式，集成了多个插件以确保代码质量：
 * - @typescript-eslint: TypeScript 静态分析
 * - eslint-plugin-import: 模块导入规范
 * - eslint-plugin-regexp: 正则表达式最佳实践
 * - eslint-plugin-security: 安全漏洞检测
 * - eslint-plugin-sonarjs: 代码异味检测
 * - eslint-plugin-unicorn: 现代 JavaScript 最佳实践
 * - eslint-plugin-vue: Vue.js 规范
 * - eslint-plugin-svelte: Svelte 规范
 * - eslint-plugin-sort-class-members: 类成员排序（支持自动修复）
 *
 * 配置文件结构：
 * - eslint/rules/: 规则配置模块
 * - eslint/custom-rules/: 自定义规则实现
 */

import fs from 'node:fs'
import path from 'node:path'
import globals from 'globals'
import js from '@eslint/js'
import { defineConfig } from 'eslint/config'
import eslintConfigPrettier from 'eslint-config-prettier/flat'

import tsEslintPlugin from '@typescript-eslint/eslint-plugin'
import tsEslintParser from '@typescript-eslint/parser'

import eslintCommentsPlugin from 'eslint-plugin-eslint-comments'
import importPlugin from 'eslint-plugin-import'
import regexpPlugin from 'eslint-plugin-regexp'
import securityPlugin from 'eslint-plugin-security'
import sonarjsPlugin from 'eslint-plugin-sonarjs'
import eslintPluginSvelte from 'eslint-plugin-svelte'
import vuePlugin from 'eslint-plugin-vue'
import sortClassMembers from 'eslint-plugin-sort-class-members'
import unicornPlugin from 'eslint-plugin-unicorn'

import svelteEslintParser from 'svelte-eslint-parser'
import vueParser from 'vue-eslint-parser'

// ----- 规则配置导入 -----
import { disabledRules, regexpRules, classMembersOrder, nestjsRules } from './eslint/rules/index.js'
import { noIsolatedComments } from './eslint/custom-rules/index.js'

/**
 * 加载 unplugin-auto-import 生成的 ESLint 全局变量配置
 *
 * 该文件由 apps/admin/vite-config/auto-import-vue.ts 中的 eslintrc 选项自动生成，
 * 包含所有通过 unplugin-auto-import 自动导入的 Vue/Pinia/VueRouter API。
 *
 * 首次使用前需要运行一次 admin 项目的开发服务器来生成此文件：
 *   pnpm --filter admin dev
 *
 * 如果文件不存在，将使用空对象，不会影响 ESLint 运行。
 */
function loadAutoImportGlobals() {
  const configPath = path.resolve(import.meta.dirname, 'apps/admin/.eslintrc-auto-import.json')
  try {
    // eslint-disable-next-line security/detect-non-literal-fs-filename
    if (fs.existsSync(configPath)) {
      // eslint-disable-next-line security/detect-non-literal-fs-filename
      const content = fs.readFileSync(configPath, 'utf-8')
      const config = JSON.parse(content)
      return config.globals || {}
    }
  } catch {
    console.warn('[ESLint] 无法加载 auto-import 全局变量配置，使用空配置')
  }
  return {}
}

const autoImportGlobals = loadAutoImportGlobals()

export default defineConfig([
  // ----- 基础配置 -----
  js.configs.recommended, // ESLint 官方推荐的 JS 配置
  sortClassMembers.configs['flat/recommended'], // 类成员排序插件默认配置

  // ----- 忽略目录 -----
  {
    ignores: [
      '**/dist/', // 构建输出
      '**/public/', // 静态资源
      '**/target/', // Rust/Tauri 构建输出
      '**/.svelte-kit/', // SvelteKit 构建缓存
      '**/node_modules/', // 依赖目录
      'packages/**', // 共享包（独立配置）
      'apps/docs/**', // 文档站点（独立配置）
      'vitest.config.ts', // Vitest 配置文件
      'eslint/**', // ESLint 配置模块
    ],
  },

  // 全局配置（作用于所有文件）
  {
    plugins: {
      regexp: regexpPlugin,
      import: importPlugin,
      sonarjs: sonarjsPlugin,
      security: securityPlugin,
      unicorn: unicornPlugin,
    },
    languageOptions: {
      ecmaVersion: 'latest',
      sourceType: 'module',
      /**
       * 全局变量配置
       * - globals.browser: 浏览器环境全局变量（window, document 等）
       * - globals.node: Node.js 环境全局变量（process, __dirname 等）
       * - autoImportGlobals: unplugin-auto-import 自动导入的 API（ref, computed 等）
       *
       * 注意：autoImportGlobals 由 apps/admin/.eslintrc-auto-import.json 自动生成，
       * 无需手动维护 Vue/Pinia/Element Plus 的 API 列表。
       */
      globals: { ...globals.browser, ...globals.node, ...autoImportGlobals },
    },
    rules: {
      // 引入推荐规则集
      ...sonarjsPlugin.configs.recommended.rules,
      ...securityPlugin.configs.recommended.rules,
      ...unicornPlugin.configs.recommended.rules,
      ...disabledRules,
      ...regexpRules,

      // ----- 代码质量 -----
      'no-debugger': 'error', // 禁止 debugger 语句
      eqeqeq: ['error', 'always', { null: 'ignore' }], // 强制全等，null 除外
      'no-var': 'error', // 禁止 var，使用 let/const
      'prefer-const': ['error', { destructuring: 'all' }], // 优先 const（可自动修复）
      'prefer-spread': 'error', // 优先展开运算符
      'prefer-template': 'error', // 优先模板字符串（可自动修复）
      'prefer-arrow-callback': 'error', // 优先箭头函数回调（可自动修复）
      'object-shorthand': ['error', 'always', { avoidQuotes: true }], // 对象简写（可自动修复）
      'one-var': ['error', 'never'], // 每个变量单独声明
      curly: 'error', // 条件语句必须使用大括号（可自动修复）
      'no-empty': ['error', { allowEmptyCatch: true }], // 禁止空代码块（catch 除外）

      // ----- 代码风格（可自动修复）-----
      semi: ['error', 'never'], // 不使用分号
      quotes: ['error', 'single', { avoidEscape: true, allowTemplateLiterals: false }], // 单引号
      'no-floating-decimal': 'error', // 数值必须有完整小数点
      'lines-between-class-members': ['error', 'always', { exceptAfterSingleLine: true }], // 类成员间空行

      // ----- 错误预防 -----
      'no-unreachable': 'error', // 禁止不可达代码
      'no-caller': 'error', // 禁止 arguments.caller/callee
      'no-eval': 'error', // 禁止 eval
      'no-implied-eval': 'error', // 禁止隐式 eval
      'no-extend-native': 'error', // 禁止扩展原生对象
      'no-extra-bind': 'error', // 禁止多余的 bind（可自动修复）
      'no-div-regex': 'error', // 避免正则歧义
      'no-restricted-syntax': ['error', 'LabeledStatement', 'WithStatement'], // 禁止特定语法
      'no-template-curly-in-string': 'error', // 检测字符串中的模板语法
      'no-unsafe-negation': 'error', // 禁止错误的否定运算
      'array-callback-return': 'error', // 数组方法必须 return

      // ----- 最佳实践 -----
      'no-alert': 'error', // 禁止 alert/confirm/prompt
      'no-iterator': 'error', // 禁止 __iterator__
      'no-labels': 'error', // 禁止标签语句
      'no-loop-func': 'error', // 禁止循环中定义函数
      'no-new-func': 'error', // 禁止 Function 构造函数
      'no-new-wrappers': 'error', // 禁止原始值包装器
      'no-proto': 'error', // 禁止 __proto__
      'no-restricted-properties': 'error', // 禁止特定属性
      'no-return-assign': 'error', // 禁止 return 中赋值
      'no-self-compare': 'error', // 禁止自比较
      'no-sequences': 'error', // 禁止逗号运算符
      'no-throw-literal': 'error', // 只能抛出 Error 对象
      'no-unmodified-loop-condition': 'error', // 循环条件必须变化
      'no-useless-call': 'error', // 禁止无用的 call/apply
      'no-useless-concat': 'error', // 禁止无用的字符串拼接
      'no-useless-return': 'error', // 禁止无用的 return（可自动修复）
      radix: 'error', // parseInt 必须指定进制
      'wrap-iife': 'error', // IIFE 必须包裹（可自动修复）
      yoda: 'error', // 禁止 Yoda 条件（可自动修复）

      // ----- 代码复杂度 -----
      'max-depth': 'error', // 限制嵌套深度（默认 4）
      'max-params': ['error', 4], // 函数最多 4 个参数
      'no-lonely-if': 'error', // 禁止孤立 if（可自动修复）
      'no-nested-ternary': 'error', // 禁止嵌套三元
      'func-style': ['error', 'declaration', { allowArrowFunctions: true }], // 函数声明风格
      'no-array-constructor': 'error', // 禁止 Array 构造函数
      'no-multi-assign': 'error', // 禁止链式赋值
      'no-new-object': 'error', // 禁止 new Object()
      'no-unneeded-ternary': 'error', // 禁止多余三元（可自动修复）

      // ----- Import 规则 -----
      'import/first': 'error', // import 必须在文件开头
      'import/no-absolute-path': 'error', // 禁止绝对路径导入
      'import/no-amd': 'error', // 禁止 AMD 语法
      'import/no-deprecated': 'off', // 允许导入已弃用模块
      'import/no-mutable-exports': 'error', // 禁止可变导出
      'import/newline-after-import': 'error', // import 后换行（可自动修复）
      'import/order': [
        'error',
        {
          groups: [
            'builtin', // Node.js 内置模块
            'external', // 外部依赖
            'internal', // 项目内部模块
            'parent', // 父级目录
            'sibling', // 同级目录
            'index', // 当前目录
            'object', // 对象导入
            'type', // 类型导入
          ],
          pathGroups: [
            { pattern: '@seed/**', group: 'internal', position: 'before' },
            { pattern: '@/**', group: 'internal', position: 'after' },
          ],
          pathGroupsExcludedImportTypes: ['type'],
        },
      ],
    },
    settings: {
      regexp: { allowedCharacterRanges: ['alphanumeric', 'а-я', 'А-Я'] },
      'import/ignore': ['node_modules'],
    },
  },

  // ----- TypeScript 文件配置 -----
  {
    files: ['**/*.ts'],
    plugins: {
      '@typescript-eslint': tsEslintPlugin,
      'eslint-comments': eslintCommentsPlugin,
      // 自定义规则插件（仅用于 TypeScript 文件）
      'custom-rules': { rules: { 'no-isolated-comments': noIsolatedComments } },
    },
    languageOptions: {
      globals: { ...globals.browser },
      parser: tsEslintParser,
      /**
       * TypeScript 项目服务配置
       *
       * projectService: true 启用 typescript-eslint 的项目服务模式，这是推荐的高性能配置方式。
       *
       * 工作原理：
       * - 自动发现并使用最近的 tsconfig.json 进行类型检查
       * - 使用持久化的 TypeScript 语言服务，避免重复解析
       * - 支持跨文件类型推断和类型感知规则
       *
       * 注意事项：
       * 1. 确保每个需要 lint 的 .ts 文件都被某个 tsconfig.json 包含
       * 2. 未被 tsconfig 包含的文件会触发警告，可能无法使用类型感知规则
       * 3. 如果遇到 "file not included in any tsconfig" 错误，检查：
       *    - tsconfig.json 的 include/exclude 配置
       *    - 是否需要创建专门的 tsconfig.eslint.json
       * 4. 对于 monorepo，每个 app/package 应有自己的 tsconfig.json
       */
      parserOptions: { projectService: true },
    },
    rules: {
      // 引入推荐规则集
      ...eslintCommentsPlugin.configs.recommended.rules,
      ...tsEslintPlugin.configs.recommended.rules,
      ...tsEslintPlugin.configs['recommended-requiring-type-checking'].rules,

      // ----- 自定义规则（仅 TypeScript）-----
      'custom-rules/no-isolated-comments': 'error', // 禁止孤立注释（可自动修复）

      // ----- 关闭的规则（由 TypeScript 接管或项目需求）-----
      'no-use-before-define': 'off', // 由 @typescript-eslint 接管
      'no-loop-func': 'off', // 由 @typescript-eslint 接管
      'eslint-comments/no-unlimited-disable': 'off', // 允许全局禁用
      '@typescript-eslint/no-unused-expressions': 'off', // 允许表达式语句
      '@typescript-eslint/no-unsafe-member-access': 'off', // 允许不安全成员访问
      '@typescript-eslint/no-unsafe-assignment': 'off', // 允许不安全赋值
      '@typescript-eslint/no-unsafe-argument': 'off', // 允许 any 参数
      '@typescript-eslint/no-unsafe-call': 'off', // 允许不安全调用
      '@typescript-eslint/no-empty-function': 'off', // 允许空函数
      '@typescript-eslint/no-explicit-any': 'off', // 允许 any 类型
      '@typescript-eslint/no-unused-vars': 'off', // 由编译器检查
      '@typescript-eslint/no-this-alias': 'off', // 允许 this 别名
      '@typescript-eslint/naming-convention': 'off', // 不强制命名规范
      '@typescript-eslint/ban-ts-comment': 'off', // 允许 @ts- 注释
      '@typescript-eslint/explicit-module-boundary-types': 'off', // 不强制导出类型
      '@typescript-eslint/no-non-null-assertion': 'off', // 允许非空断言 !
      '@typescript-eslint/no-non-null-asserted-optional-chain': 'off', // 允许 ?.!
      '@typescript-eslint/no-var-requires': 'off', // 允许 require
      '@typescript-eslint/no-empty-object-type': 'off', // 允许空对象类型 {}
      '@typescript-eslint/member-ordering': 'off', // 由 sort-class-members 接管
      '@typescript-eslint/parameter-properties': 'off', // 允许构造函数参数属性
      '@typescript-eslint/prefer-nullish-coalescing': 'off', // 不强制 ??

      // ----- 类型导入（可自动修复）-----
      '@typescript-eslint/consistent-type-imports': ['error', { disallowTypeAnnotations: true }],

      // ----- Promise 规则（防止异步错误）-----
      '@typescript-eslint/no-floating-promises': 'error', // 必须处理 Promise
      '@typescript-eslint/await-thenable': 'error', // 只能 await Promise
      '@typescript-eslint/no-misused-promises': ['error', { checksVoidReturn: false }], // 防止 Promise 误用
      '@typescript-eslint/return-await': ['error', 'in-try-catch'], // try-catch 中使用 return await
      '@typescript-eslint/require-await': 'error', // async 函数必须有 await

      // ----- 代码质量 -----
      '@typescript-eslint/no-use-before-define': ['error', { functions: false, classes: false }],
      '@typescript-eslint/no-loop-func': 'error', // 禁止循环中定义函数
      '@typescript-eslint/prefer-optional-chain': 'error', // 优先可选链 ?.（可自动修复）
      '@typescript-eslint/prefer-readonly': 'error', // 不变属性声明为 readonly
      '@typescript-eslint/adjacent-overload-signatures': 'error', // 重载签名放一起

      // ----- 类成员可访问性（可自动修复）-----
      '@typescript-eslint/explicit-member-accessibility': [
        'error',
        {
          accessibility: 'explicit',
          overrides: {
            accessors: 'no-public', // getter/setter 省略 public
            constructors: 'no-public', // 构造函数省略 public
            methods: 'no-public', // 方法省略 public
            properties: 'no-public', // 属性省略 public
            parameterProperties: 'no-public', // 参数属性省略 public
          },
        },
      ],

      // ----- 类成员排序（可自动修复）-----
      'sort-class-members/sort-class-members': [
        'error',
        { order: classMembersOrder, accessorPairPositioning: 'getThenSet' },
      ],

      // ----- ESLint 注释规则 -----
      'eslint-comments/disable-enable-pair': ['error', { allowWholeFile: true }],
    },
  },

  // ----- Vue 文件配置 -----
  {
    files: ['**/*.vue'],
    plugins: { vue: vuePlugin },
    languageOptions: {
      globals: { ...globals.browser },
      parser: vueParser,
      parserOptions: { ecmaVersion: 'latest', sourceType: 'module', parser: tsEslintParser },
    },
    rules: {
      ...vuePlugin.configs['flat/recommended'].rules, // Vue 官方推荐配置

      // ----- 关闭的规则（根据项目需求）-----
      'vue/no-deprecated-slot-attribute': 'off', // 允许旧版 slot 属性
      'vue/prefer-import-from-vue': 'off', // 允许从其他路径导入
      'vue/multi-word-component-names': 'off', // 允许单词组件名
      'vue/valid-define-props': 'off', // 不校验 defineProps
      'vue/no-v-model-argument': 'off', // 允许 v-model 参数
      'vue/no-reserved-component-names': 'off', // 允许保留组件名
      'vue/require-default-prop': 'off', // 不强制 props 默认值
      'vue/require-explicit-emits': 'off', // 不强制显式 emits
      'vue/no-template-shadow': 'off', // 允许模板变量遮蔽
      'vue/no-mutating-props': 'off', // 允许修改 props
      'vue/require-valid-default-prop': 'off', // 不校验默认值类型
      'vue/no-unused-vars': 'off', // 允许模板中未使用变量
      'vue/no-v-html': 'off', // 允许 v-html
      'no-unused-vars': 'off', // Vue 文件中关闭
    },
  },

  // ----- ESLint 配置文件自身 -----
  { files: ['eslint.config.js'], languageOptions: { globals: { ...globals.node } } },

  // ----- Svelte 文件配置 -----
  {
    files: ['**/*.svelte'],
    plugins: { svelte: eslintPluginSvelte },
    languageOptions: { globals: { ...globals.browser }, parser: svelteEslintParser },
    rules: { ...eslintPluginSvelte.configs.recommended.rules },
  },

  // ----- NestJS API 服务器 -----
  {
    files: ['apps/api-server/**/*.ts'],
    rules: { ...nestjsRules, 'unicorn/no-anonymous-default-export': 'off' },
  },
  {
    files: ['apps/server-api/**/*.ts'],
    rules: { ...nestjsRules, 'unicorn/no-anonymous-default-export': 'off' },
  },

  // ----- Tauri 更新服务器 -----
  { files: ['apps/tauri-updater/**/*.ts'], rules: nestjsRules },

  // ----- 代理服务器和静态服务器 -----
  {
    files: ['apps/server-proxy/**/*.ts', 'apps/server-static/**/*.ts'],
    rules: { 'unicorn/prefer-top-level-await': 'off', 'unicorn/no-process-exit': 'off' },
  },

  // ----- 测试文件配置 -----
  {
    files: [
      '**/*.spec.ts',
      '**/*.test.ts',
      '**/*.spec.js',
      '**/*.test.js',
      'apps/server-api/test/**/*.ts',
    ],
    languageOptions: {
      globals: {
        // Jest/Vitest 全局变量
        describe: 'readonly',
        it: 'readonly',
        test: 'readonly',
        expect: 'readonly',
        jest: 'readonly',
        vi: 'readonly',
        beforeEach: 'readonly',
        afterEach: 'readonly',
        beforeAll: 'readonly',
        afterAll: 'readonly',
      },
    },
    rules: {
      'unicorn/no-useless-undefined': 'off', // 测试中允许 undefined mock
      '@typescript-eslint/no-floating-promises': 'off', // 测试中放宽 Promise 规则
      '@typescript-eslint/no-misused-promises': 'off',
    },
  },

  // ----- Prettier 配置覆盖（必须放在最后）-----
  eslintConfigPrettier,
])
