import globals from 'globals'
import js from '@eslint/js'
import { defineConfig } from 'eslint/config'
import eslintConfigPrettier from 'eslint-config-prettier/flat'

import tsEslintPlugin from '@typescript-eslint/eslint-plugin'
import tsEslintParser from '@typescript-eslint/parser'

import eslintCommentsPlugin from 'eslint-plugin-eslint-comments'
import importPlugin from 'eslint-plugin-import'
import nodePlugin from 'eslint-plugin-node'
import promisePlugin from 'eslint-plugin-promise'
import regexpPlugin from 'eslint-plugin-regexp'
import securityPlugin from 'eslint-plugin-security'
import sonarjsPlugin from 'eslint-plugin-sonarjs'
import eslintPluginSvelte from 'eslint-plugin-svelte'
import vuePlugin from 'eslint-plugin-vue'
import sortClassMembers from 'eslint-plugin-sort-class-members'
import unicornPlugin from 'eslint-plugin-unicorn'

import svelteEslintParser from 'svelte-eslint-parser'
import vueParser from 'vue-eslint-parser'

export default defineConfig([
  js.configs.recommended, // 使用 ESLint 官方推荐的 JS 配置
  sortClassMembers.configs['flat/recommended'],
  {
    // 指定需要忽略的目录
    ignores: [
      '**/dist/',
      '**/public/',
      '**/.svelte-kit/',
      '**/node_modules/',
      './packages/',
      './apps/docs/',
      './apps/admin/src/components/VvDropdown/',
    ],
  },
  {
    // “全局”配置，作用于所有文件
    plugins: {
      regexp: regexpPlugin,
      import: importPlugin,
      promise: promisePlugin,
      node: nodePlugin,
      sonarjs: sonarjsPlugin,
      security: securityPlugin,
      unicorn: unicornPlugin,
    },
    languageOptions: {
      ecmaVersion: 'latest',
      sourceType: 'module',
      globals: {
        ...globals.browser,
        ...globals.node,
        ...vue3Globals(),
        ...elementPlusGlobals(),
      },
    },
    rules: {
      ...sonarjsPlugin.configs.recommended.rules, // 引入 SonarJS 推荐规则集
      ...securityPlugin.configs.recommended.rules, // 引入 Security 推荐规则集
      ...unicornPlugin.configs.recommended.rules,
      // 关闭 SonarJS 部分过于严格的检查
      'sonarjs/pseudo-random': 'off', // 允许使用 Math.random
      'sonarjs/no-ignored-exceptions': 'off', // 允许空的 catch 块
      'sonarjs/todo-tag': 'off', // 允许无 issue 的 TODO 注释
      'sonarjs/no-unused-vars': 'off', // 允许未使用变量
      'sonarjs/no-dead-store': 'off', // 允许无效赋值
      'sonarjs/cognitive-complexity': 'off', // 关闭复杂度警告
      'sonarjs/no-hardcoded-ip': 'off', // 允许硬编码 IP
      'sonarjs/no-hardcoded-passwords': 'off', // 允许硬编码密码
      'sonarjs/slow-regex': 'off', // 允许慢正则
      'sonarjs/prefer-regexp-exec': 'off', // 允许 String.match
      'sonarjs/unused-import': 'off', // 允许未使用的 import

      // 关闭安全检测中对对象注入的限制
      'security/detect-object-injection': 'off', // 允许动态属性访问

      // 关闭 Unicorn 插件的代码风格限制
      'unicorn/no-abusive-eslint-disable': 'off', // 允许不指定规则全局禁用 ESLint
      'unicorn/filename-case': 'off', // 不强制文件名 kebab-case
      'unicorn/no-null': 'off', // 允许使用 null
      'unicorn/prevent-abbreviations': 'off', // 允许缩写变量名
      'unicorn/prefer-logical-operator-over-ternary': 'off', // 允许简单三元
      'unicorn/prefer-spread': 'off', // 允许使用 split/apply
      'unicorn/prefer-number-properties': 'off', // 允许全局 isNaN/isFinite
      'unicorn/consistent-function-scoping': 'off', // 允许嵌套函数
      'unicorn/no-invalid-remove-event-listener': 'off', // 关闭 removeEventListener 参数检查
      'unicorn/prefer-add-event-listener': 'off', // 允许 onmessage/onerror
      'unicorn/prefer-single-call': 'off', // 允许多次 push
      'unicorn/no-useless-switch-case': 'off', // 允许无意义的 switch case
      'unicorn/prefer-code-point': 'off', // 允许 fromCharCode/charCodeAt
      'unicorn/no-useless-spread': 'off', // 允许无用的 spread
      'unicorn/prefer-blob-reading-methods': 'off', // 允许 FileReader
      'unicorn/prefer-structured-clone': 'off', // 允许 JSON deep clone
      'unicorn/import-style': 'off', // 不强制某种 import 语法
      'unicorn/text-encoding-identifier-case': 'off', // 允许 utf-8 标识
      'unicorn/no-object-as-default-parameter': 'off', // 允许对象字面量默认值
      'unicorn/prefer-ternary': 'off', // 允许 if-else
      'unicorn/prefer-regexp-test': 'off', // 允许 String.match

      // 关闭 ESLint 核心规则的部分警告
      'no-lone-blocks': 'off', // 允许多余的嵌套块
      'no-void': 'off', // 允许 void 运算符
      'no-unused-vars': 'off', // 关闭未使用变量检查
      'no-prototype-builtins': 'off', // 允许直接调用 Object.prototype
      'no-inner-declarations': 'off', // 允许函数内声明
      'no-control-regex': 'off', // 允许正则控制字符
      'no-empty-character-class': 'off', // 允许空字符类
      'no-useless-escape': 'off', // 允许多余转义
      'complexity': 'off', // 关闭复杂度限制
      'max-lines': 'off', // 关闭长度限制

      'no-use-before-define': ['warn', { functions: false, classes: false }], // 禁止在声明前使用，防止引用错误
      'eqeqeq': ['warn', 'always', { null: 'ignore' }], // 强制使用全等避免隐式类型转换
      'no-var': 'warn', // 不允许使用 var 声明
      'object-shorthand': ['warn', 'always', { avoidQuotes: true }], // 强制对象字面量使用速记语法
      'one-var': ['warn', 'never'], // 禁止使用一条声明多个变量
      'prefer-arrow-callback': 'warn', // 优先使用箭头函数回调
      'prefer-const': ['warn', { destructuring: 'all' }], // 建议使用 const 声明只读变量
      'prefer-spread': 'warn', // 建议使用扩展运算符而非 apply调用
      'curly': 'warn', // 在条件语句中强制使用大括号
      'semi': ['warn', 'never'], // 不允许语句末尾有分号
      'quotes': ['warn', 'single', { avoidEscape: true, allowTemplateLiterals: false }], // 强制使用单引号
      'no-restricted-syntax': ['warn', 'LabeledStatement', 'WithStatement'], // 禁止使用部分易混淆语法
      'no-empty': ['warn', { allowEmptyCatch: true }], // 禁止空代码块
      'prefer-template': 'warn', // 建议使用模板字符串
      'no-unreachable': 'warn', // 禁止在可达不到的代码中继续编写
      'no-caller': 'warn', // 禁止使用 arguments.caller 或 arguments.callee
      'no-div-regex': 'warn', // 警告可能错误的正则表达式分隔符
      'no-eval': 'warn', // 避免使用 eval
      'no-extend-native': 'warn', // 禁止扩展原生对象
      'no-extra-bind': 'warn', // 禁止不必要的 bind
      'no-floating-decimal': 'warn', // 数值不能脱离小数点
      'lines-between-class-members': ['warn', 'always', { exceptAfterSingleLine: true }], // 强制类成员之间的空行
      'import/first': 'warn', // 可以在 import 之前编写其他语句
      'import/no-absolute-path': 'warn', // 禁止从绝对路径导入
      'import/no-amd': 'warn', // 禁止使用 AMD 模块语法
      'import/no-deprecated': 'warn', // 禁止导入已被弃用的模块
      'import/no-mutable-exports': 'warn', // 禁止可变的导出变量
      'import/newline-after-import': 'warn', // import 后需有换行
      'import/order': [
        'warn',
        {
          groups: [
            // 内置模块
            'builtin',
            // 外部依赖
            'external',
            // 项目内部模块
            'internal',
            // 父级目录导入
            'parent',
            // 同级目录导入
            'sibling',
            // 入口文件或当前目录
            'index',
            // 对象方式导入
            'object',
            // 类型导入
            'type',
          ],
          pathGroups: [
            { pattern: '@roboui/**', group: 'internal', position: 'before' },
            { pattern: '@/**', group: 'internal', position: 'after' },
          ],
          pathGroupsExcludedImportTypes: ['type'],
        },
      ],
      'regexp/no-dupe-disjunctions': 'warn', // 禁止在正则中重复筛选模式
      'regexp/no-empty-alternative': 'warn', // 正则的分组替代不得为空
      'regexp/no-empty-capturing-group': 'warn', // 禁止空捕获组
      'regexp/no-empty-lookarounds-assertion': 'warn', // 禁止空的环视断言
      'regexp/no-lazy-ends': 'warn', // 禁止在不必要的地方使用惰性量词
      'regexp/no-obscure-range': 'warn', // 禁止低可读性的字符范围
      'regexp/no-optional-assertion': 'warn', // 禁止使用可选断言
      'regexp/no-standalone-backslash': 'warn', // 禁止孤立的反斜杠
      'regexp/no-super-linear-backtracking': 'warn', // 防止超级线性回溯风险
      'regexp/no-unused-capturing-group': 'warn', // 禁止未使用的捕获组
      'regexp/no-zero-quantifier': 'warn', // 禁止无效的零宽量词
      'regexp/optimal-lookaround-quantifier': 'warn', // 优化 lookaround 量词使用
      'regexp/match-any': 'warn', // 规范正则中任意字符的写法
      'regexp/negation': 'warn', // 规范否定匹配
      'regexp/no-dupe-characters-character-class': 'warn', // 字符类中不允许重复字符
      'regexp/no-trivially-nested-assertion': 'warn', // 防止无意义的断言嵌套
      'regexp/no-trivially-nested-quantifier': 'warn', // 防止无意义的量词嵌套
      'regexp/no-useless-character-class': 'warn', // 消除无用的字符类
      'regexp/no-useless-flag': 'warn', // 不要使用无意义的正则标志
      'regexp/no-useless-lazy': 'warn', // 不要使用无意义的惰性量词
      'regexp/no-useless-range': 'warn', // 禁止多余的字符区间
      'regexp/prefer-d': ['warn', { insideCharacterClass: 'ignore' }], // 优先使用 \d
      'regexp/prefer-plus-quantifier': 'warn', // 优先使用 + 而不是 {1,}
      'regexp/prefer-question-quantifier': 'warn', // 优先使用 ? 而不是 {0,1}
      'regexp/prefer-star-quantifier': 'warn', // 优先使用 * 而不是 {0,}
      'regexp/prefer-w': 'warn', // 优先使用 \w
      'regexp/sort-alternatives': 'warn', // 使用有意义的排序方式管理正则分组
      'regexp/sort-flags': 'warn', // 排序正则标志以提高可读性
      'regexp/strict': 'warn', // 鼓励更严格的正则规范

      'max-depth': 'warn', // 限制代码块嵌套深度
      'max-params': ['warn', 4], // 函数最多允许四个参数
      'no-lonely-if': 'warn', // 禁止孤立的 if 块
      'no-nested-ternary': 'warn', // 禁用嵌套三元表达式
      'func-style': ['warn', 'declaration', { allowArrowFunctions: true }], // 使用声明式函数
      'no-array-constructor': 'warn', // 禁止使用 Array 构造函数
      'no-bitwise': 'warn', // 避免使用位操作符
      'no-multi-assign': 'warn', // 禁止链式赋值
      'no-new-object': 'warn', // 禁止 new Object()
      'no-unneeded-ternary': 'warn', // 禁止不必要的三元运算
      'no-template-curly-in-string': 'warn', // 禁止在字符串中出现模板特殊字符
      'no-unsafe-negation': 'warn', // 禁止对关系运算符的错误使用
      'array-callback-return': 'warn', // 回调必须有 return
      'consistent-return': 'warn', // 函数分支返回值类型要一致
      'guard-for-in': 'warn', // 需要在 for-in 中检查 hasOwnProperty
      'no-alert': 'warn', // 警告使用 alert
      'no-implied-eval': 'warn', // 禁止隐式调用 eval
      'no-iterator': 'warn', // 禁止使用 __iterator__
      'no-labels': 'warn', // 禁用标签语句
      'no-loop-func': 'warn', // 禁止在循环中定义函数
      'no-new': 'warn', // 禁用不存储结果的 new 操作
      'no-new-func': 'warn', // 禁止接受字符串作为参数的 Function 构造函数
      'no-new-wrappers': 'warn', // 避免 String/Number/Boolean 对象包装
      'no-proto': 'warn', // 禁用 __proto__ 属性
      'no-restricted-properties': 'warn', // 禁止使用某些对象属性
      'no-return-assign': 'warn', // 禁止在 return 中赋值
      'no-return-await': 'warn', // 禁止在 return 中使用 await
      'no-self-compare': 'warn', // 禁止与自身比较
      'no-sequences': 'warn', // 禁止使用逗号运算符
      'no-throw-literal': 'warn', // 强制抛出 Error 对象
      'no-unmodified-loop-condition': 'warn', // 循环条件禁止始终不变
      'no-unused-expressions': ['warn', { allowShortCircuit: true }], // 禁止无用的表达式
      'no-useless-call': 'warn', // 禁止不必要的 call 或 apply
      'no-useless-concat': 'warn', // 禁止没必要的字符串拼接
      'no-useless-return': 'warn', // 禁止不必要的 return
      'radix': 'warn', // parseInt 必须指定进制
      'require-await': 'warn', // async 函数内必须有 await
      'wrap-iife': 'warn', // 立即执行函数需要用括号包裹
      'yoda': 'warn', // 避免在条件判断中使用字面量在左侧的比较
      'consistent-this': ['warn', 'that'], // this 需被赋值给 that
    },
    settings: {
      regexp: {
        allowedCharacterRanges: ['alphanumeric', 'а-я', 'А-Я'], // 允许在正则中使用的字符范围
      },
      'import/ignore': ['node_modules'], // 忽略指定目录的 import
    },
  },
  {
    files: ['**/*.ts'],
    plugins: {
      '@typescript-eslint': tsEslintPlugin,
      'eslint-comments': eslintCommentsPlugin,
      'regexp': regexpPlugin,
    },
    languageOptions: {
      globals: {
        ...globals.browser,
      },
      parser: tsEslintParser,
      parserOptions: {
        projectService: true, // 允许基于当前项目进行类型检查
      },
    },
    rules: {
      ...eslintCommentsPlugin.configs.recommended.rules, // 使用 ESLint 注释规则推荐配置
      ...tsEslintPlugin.configs.recommended.rules, // 使用 TS 推荐规则
      ...tsEslintPlugin.configs['recommended-requiring-type-checking'].rules, // 需要类型检查的推荐规则
      ...regexpPlugin.configs.recommended.rules, // 正则相关推荐规则

      // 根据项目需求关闭部分规则（建议打开，删除即可）
      '@typescript-eslint/no-unsafe-member-access': 'off', // 允许访问不安全的成员
      '@typescript-eslint/no-unsafe-assignment': 'off', // 允许不安全的赋值

      '@typescript-eslint/no-unsafe-argument': 'off', // 允许 any/unknown 参数

      'no-use-before-define': 'off', // 允许在声明前使用（由 TS 检测）
      'eslint-comments/no-unlimited-disable': 'off', // 允许在文件开头禁用全部规则
      '@typescript-eslint/no-empty-function': 'off', // 允许空函数
      '@typescript-eslint/no-explicit-any': 'off', // 允许使用 any
      '@typescript-eslint/no-unused-vars': 'off', // 允许未使用变量（可能由各插件自行处理）
      '@typescript-eslint/no-this-alias': 'off', // 允许将 this 赋值给其他变量
      '@typescript-eslint/naming-convention': 'off', // 不强制命名规范
      '@typescript-eslint/ban-ts-comment': 'off', // 允许使用 @ts- 注释
      '@typescript-eslint/explicit-module-boundary-types': 'off', // 不强制导出函数需声明返回类型
      '@typescript-eslint/no-non-null-assertion': 'off', // 允许使用非空断言
      '@typescript-eslint/no-non-null-asserted-optional-chain': 'off', // 允许在可选链中使用非空断言
      '@typescript-eslint/no-var-requires': 'off', // 允许使用 require 导入
      '@typescript-eslint/no-empty-object-type': 'off', // 允许空对象类型
      '@typescript-eslint/consistent-type-imports': ['warn', { disallowTypeAnnotations: true }], // 建议使用统一的类型导入风格
      '@typescript-eslint/no-unsafe-call': 'off', // 允许调用不安全类型
      'no-prototype-builtins': 'off', // 允许使用 Object.prototype.* 调用
      'no-control-regex': 'off', // 允许正则中使用控制字符
      'no-empty-character-class': 'off', // 允许空字符类
      'no-useless-escape': 'off', // 允许字符串或正则中的转义

      // 规定类成员的书写顺序
      '@typescript-eslint/member-ordering': [
        'warn',
        {
          default: [
            // 1. 公共静态字段/方法
            'public-static-field',
            'public-static-readonly-field',
            'static-initialization',
            'public-static-get',
            'public-static-set',
            'public-static-method',

            // 2. 受保护静态字段/方法
            'protected-static-field',
            'protected-static-readonly-field',
            'protected-static-get',
            'protected-static-set',
            'protected-static-method',

            // 3. 私有静态字段/方法
            'private-static-field',
            'private-static-readonly-field',
            'private-static-get',
            'private-static-set',
            'private-static-method',

            // 4. 公共实例字段
            'public-instance-field',
            'public-decorated-field',
            'public-readonly-field',

            // 5. 受保护实例字段
            'protected-instance-field',
            'protected-readonly-field',

            // 6. 私有实例字段
            'private-instance-field',
            'private-readonly-field',

            // 7. 构造函数
            'constructor',

            // 8. 公共存取器
            'public-instance-get',
            'public-instance-set',

            // 9. 受保护存取器
            'protected-instance-get',
            'protected-instance-set',

            // 10. 私有存取器
            'private-instance-get',
            'private-instance-set',

            // 11. 公共实例方法
            'public-instance-method',

            // 12. 受保护实例方法
            'protected-instance-method',

            // 13. 私有实例方法
            'private-instance-method',
          ],
        },
      ],
      'sort-class-members/sort-class-members': [
        'warn',
        {
          order: [
            // 1. 公共静态字段/方法
            { type: 'property', static: true, accessibility: 'public' },
            { type: 'property', static: true, accessibility: 'public', readonly: true },
            { type: 'method', static: true, accessibility: 'public', kind: 'get' },
            { type: 'method', static: true, accessibility: 'public', kind: 'set' },
            { type: 'method', static: true, accessibility: 'public' },

            // 2. 受保护静态字段/方法
            { type: 'property', static: true, accessibility: 'protected' },
            { type: 'property', static: true, accessibility: 'protected', readonly: true },
            { type: 'method', static: true, accessibility: 'protected', kind: 'get' },
            { type: 'method', static: true, accessibility: 'protected', kind: 'set' },
            { type: 'method', static: true, accessibility: 'protected' },

            // 3. 私有静态字段/方法
            { type: 'property', static: true, accessibility: 'private' },
            { type: 'property', static: true, accessibility: 'private', readonly: true },
            { type: 'method', static: true, accessibility: 'private', kind: 'get' },
            { type: 'method', static: true, accessibility: 'private', kind: 'set' },
            { type: 'method', static: true, accessibility: 'private' },

            // 4. 公共实例字段
            { type: 'property', static: false, accessibility: 'public' },
            { type: 'property', static: false, accessibility: 'public', groupByDecorator: true },
            { type: 'property', static: false, accessibility: 'public', readonly: true },

            // 5. 受保护实例字段
            { type: 'property', static: false, accessibility: 'protected' },
            { type: 'property', static: false, accessibility: 'protected', readonly: true },

            // 6. 私有实例字段
            { type: 'property', static: false, accessibility: 'private' },
            { type: 'property', static: false, accessibility: 'private', readonly: true },

            // 7. 构造函数
            'constructor',

            // 8. 公共存取器
            { type: 'method', static: false, accessibility: 'public', kind: 'get' },
            { type: 'method', static: false, accessibility: 'public', kind: 'set' },

            // 9. 受保护存取器
            { type: 'method', static: false, accessibility: 'protected', kind: 'get' },
            { type: 'method', static: false, accessibility: 'protected', kind: 'set' },

            // 10. 私有存取器
            { type: 'method', static: false, accessibility: 'private', kind: 'get' },
            { type: 'method', static: false, accessibility: 'private', kind: 'set' },

            // 11. 公共实例方法
            { type: 'method', static: false, accessibility: 'public' },

            // 12. 受保护实例方法
            { type: 'method', static: false, accessibility: 'protected' },

            // 13. 私有实例方法
            { type: 'method', static: false, accessibility: 'private' },
          ],
          accessorPairPositioning: 'getThenSet',
        },
      ],
      '@typescript-eslint/prefer-readonly': 'warn', // 建议将不变的属性声明为只读
      // '@typescript-eslint/no-useless-constructor': 'warn', // 禁止多余的构造器
      '@typescript-eslint/explicit-member-accessibility': [
        'warn',
        {
          accessibility: 'explicit',
          // 省略 public 修饰符
          overrides: {
            // (getter/setter)
            accessors: 'no-public',
            constructors: 'no-public',
            methods: 'no-public',
            properties: 'no-public',
            parameterProperties: 'no-public',
          },
        },
      ],
      '@typescript-eslint/parameter-properties': ['warn', { prefer: 'class-property' }], // 建议使用类属性来声明参数
      '@typescript-eslint/adjacent-overload-signatures': 'warn', // 相邻重载签名需放在一起
      'eslint-comments/disable-enable-pair': ['warn', { allowWholeFile: true }], // 允许在文件整体上禁用/启用 ESLint
    },
  },
  {
    files: ['**/*.vue'],
    plugins: {
      vue: vuePlugin,
    },
    languageOptions: {
      globals: {
        ...globals.browser,
      },
      parser: vueParser,
      parserOptions: {
        ecmaVersion: 'latest',
        sourceType: 'module',
        parser: tsEslintParser,
      },
    },
    rules: {
      ...vuePlugin.configs.rules, // 使用 Vue 官方推荐配置
      'vue/no-deprecated-slot-attribute': 'off', // 允许使用已废弃的 slot 属性
      'vue/prefer-import-from-vue': 'off', // 允许直接从其他地方导入
      'vue/multi-word-component-names': 'off', // 允许单词组件名
      'vue/valid-define-props': 'off', // 不校验 defineProps 正确性
      'vue/no-v-model-argument': 'off', // 允许 v-model 传参
      'vue/no-reserved-component-names': 'off', // 允许使用保留组件名
      'vue/require-default-prop': 'off', // 不强制要求 props 配置默认值
      'vue/require-explicit-emits': 'off', // 不强制显式声明 emits
      'vue/no-template-shadow': 'off', // 允许 template 级 shadow 变量
      'vue/no-mutating-props': 'off', // 允许修改 props
      'vue/require-valid-default-prop': 'off', // 不强制校验默认值
      'vue/no-unused-vars': 'off', // 在模板中允许出现未使用变量
      'vue/no-v-html': 'off', // 允许使用 v-html
      'no-unused-vars': 'off', // 关闭在 .vue 文件内的未使用变量检测
    },
  },
  {
    files: ['eslint.config.mjs'],
    languageOptions: {
      globals: {
        ...globals.node,
      },
    },
  },
  {
    files: ['**/*.svelte'],
    plugins: {
      svelte: eslintPluginSvelte,
    },
    languageOptions: {
      globals: {
        ...globals.browser,
      },
      parser: svelteEslintParser,
    },
    rules: {
      ...eslintPluginSvelte.configs.recommended.rules, // 使用 Svelte 推荐规则
    },
  },
  {
    files: ['apps/api-server/**/*.ts'],
    rules: {
      '@typescript-eslint/parameter-properties': 'off',
      '@typescript-eslint/no-base-to-string': 'off',
      'unicorn/prefer-top-level-await': 'off',
      'unicorn/no-process-exit': 'off',
      'unicorn/prefer-module': 'off',
    },
  },
  eslintConfigPrettier, // 使用 Prettier 配置覆盖部分格式化规则
])

// Vue 3 核心 API 全局变量
function vue3Globals() {
  return {
    // ===== 应用 API =====
    createApp: true, // 创建应用实例

    // ===== 基础响应式 API =====
    ref: true, // 创建响应式引用
    reactive: true, // 创建深度响应式对象
    readonly: true, // 创建只读代理
    shallowRef: true, // 创建浅层响应式引用
    shallowReactive: true, // 创建浅层响应式对象
    shallowReadonly: true, // 创建浅层只读对象

    // ===== 响应式工具 =====
    isRef: true, // 检查值是否为 ref 对象
    isReactive: true, // 检查对象是否由 reactive() 创建
    isReadonly: true, // 检查对象是否由 readonly() 创建
    isProxy: true, // 检查对象是否是响应式代理
    unref: true, // 返回 ref 的值或参数本身
    toRef: true, // 将响应式对象的属性转为 ref
    toRefs: true, // 将响应式对象转为普通对象，每个属性都是 ref
    toRaw: true, // 返回代理对应的原始对象
    markRaw: true, // 标记对象为不可转为代理

    // ===== 计算属性与侦听器 =====
    computed: true, // 创建计算属性
    watch: true, // 侦听响应式数据源
    watchEffect: true, // 立即运行函数并响应式追踪依赖
    watchPostEffect: true, // DOM 更新后执行的 watchEffect
    watchSyncEffect: true, // 同步执行的 watchEffect

    // ===== 生命周期钩子 =====
    onBeforeMount: true, // 组件挂载前调用
    onMounted: true, // 组件挂载后调用
    onBeforeUpdate: true, // 组件更新前调用
    onUpdated: true, // 组件更新后调用
    onBeforeUnmount: true, // 组件卸载前调用
    onUnmounted: true, // 组件卸载后调用
    onActivated: true, // KeepAlive 组件激活时调用
    onDeactivated: true, // KeepAlive 组件停用时调用
    onErrorCaptured: true, // 捕获后代组件错误时调用
    onRenderTracked: true, // 响应式依赖被收集时调用
    onRenderTriggered: true, // 响应式依赖被触发时调用
    onServerPrefetch: true, // 服务端渲染前调用

    // ===== 依赖注入 =====
    provide: true, // 提供值给后代组件
    inject: true, // 注入祖先组件提供的值

    // ===== 组件定义 =====
    defineComponent: true, // 定义组件并提供类型推导
    defineAsyncComponent: true, // 定义异步组件

    // ===== 渲染函数 =====
    h: true, // 创建虚拟 DOM 节点
    resolveComponent: true, // 解析已注册的组件
    resolveDirective: true, // 解析已注册的指令

    // ===== 高级 API =====
    getCurrentInstance: true, // 获取当前组件实例
    getCurrentScope: true, // 获取当前 effect 作用域
    effectScope: true, // 创建 effect 作用域
    onScopeDispose: true, // 作用域销毁时的回调
    customRef: true, // 创建自定义 ref
    triggerRef: true, // 强制触发 ref 的副作用
    nextTick: true, // 等待下次 DOM 更新

    // ===== 路由（Vue Router）=====
    useRouter: true, // 获取路由器实例
    useRoute: true, // 获取当前路由信息

    // ===== 状态管理（Pinia）=====
    storeToRefs: true, // 将 store 转换为响应式 refs

    // ===== TypeScript 类型 =====
    ComponentInternalInstance: true, // 组件内部实例类型
    EffectScope: true, // effect 作用域类型
    PropType: true, // 组件 props 类型定义
  }
}

// Element Plus UI 组件库全局变量
function elementPlusGlobals() {
  return {
    // ===== 组件类型定义 =====
    // 表单相关类型
    ElFormRules: true, // 表单验证规则类型
    CheckboxValueType: true, // 复选框值类型
    DateModelType: true, // 日期选择器数据类型
    UploadFile: true, // 上传文件类型

    // ===== 组件实例类型 =====
    // 表单组件实例
    ElFormInstance: true, // 表单组件实例类型
    ElInputInstance: true, // 输入框组件实例类型
    ElInputNumberInstance: true, // 数字输入框组件实例类型

    // 选择器组件实例
    ElSelectInstance: true, // 选择器组件实例类型
    ElTreeSelectInstance: true, // 树形选择器组件实例类型
    ElCascaderInstance: true, // 级联选择器组件实例类型

    // 单选/复选组件实例
    ElRadioInstance: true, // 单选框组件实例类型
    ElRadioGroupInstance: true, // 单选框组组件实例类型
    ElRadioButtonInstance: true, // 单选按钮组件实例类型
    ElCheckboxInstance: true, // 复选框组件实例类型
    ElCheckboxGroupInstance: true, // 复选框组组件实例类型
    ElSwitchInstance: true, // 开关组件实例类型

    // 日期时间组件实例
    ElDatePickerInstance: true, // 日期选择器组件实例类型
    ElTimePickerInstance: true, // 时间选择器组件实例类型
    ElTimeSelectInstance: true, // 时间选择组件实例类型

    // 数据展示组件实例
    ElTableInstance: true, // 表格组件实例类型
    ElTreeInstance: true, // 树形控件组件实例类型

    // 上传组件实例
    ElUploadInstance: true, // 上传组件实例类型

    // 布局容器组件实例
    ElCardInstance: true, // 卡片组件实例类型
    ElDialogInstance: true, // 对话框组件实例类型
    ElScrollbarInstance: true, // 滚动条组件实例类型

    // 其他组件实例
    ElColorPickerInstance: true, // 颜色选择器组件实例类型
    ElRateInstance: true, // 评分组件实例类型
    ElSliderInstance: true, // 滑块组件实例类型

    // ===== 常用组件引用 =====
    ElTable: true, // 表格组件
    ElSelect: true, // 选择器组件
    ElUpload: true, // 上传组件
    ElForm: true, // 表单组件
    ElTree: true, // 树形控件组件

    // ===== 消息提示 =====
    ElMessage: true, // 消息提示
    ElMessageBox: true, // 消息弹框

    // ===== 项目自定义类型 =====
    TransferKey: true, // 穿梭框键值类型
    ImportOption: true, // 导入选项类型
    TreeType: true, // 树形数据类型
    FieldOption: true, // 字段选项类型
    PageData: true, // 分页数据类型
    DictDataOption: true, // 字典数据选项类型
    UploadOption: true, // 上传选项类型
  }
}
