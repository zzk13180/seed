/**
 * 根据项目需求关闭的规则
 * 这些规则要么过于严格，要么与项目实际情况不符
 */
export const disabledRules = {
  // ----- SonarJS 规则 -----
  'sonarjs/pseudo-random': 'off', // 允许使用 Math.random（非加密场景）
  'sonarjs/no-ignored-exceptions': 'off', // 允许空的 catch 块
  'sonarjs/todo-tag': 'off', // 允许无 issue 编号的 TODO 注释
  'sonarjs/no-unused-vars': 'off', // 由 TypeScript 编译器检查
  'sonarjs/no-dead-store': 'off', // 允许无效赋值（某些调试场景需要）
  'sonarjs/cognitive-complexity': 'off', // 关闭认知复杂度限制
  'sonarjs/no-hardcoded-ip': 'off', // 允许硬编码 IP（开发环境配置）
  'sonarjs/no-hardcoded-passwords': 'off', // 允许硬编码密码（测试数据）
  'sonarjs/slow-regex': 'off', // 允许可能较慢的正则表达式
  'sonarjs/prefer-regexp-exec': 'off', // 允许使用 String.match
  'sonarjs/unused-import': 'off', // 由其他工具处理
  'sonarjs/no-commented-code': 'off', // 允许注释掉的代码（临时保留）

  // ----- Security 规则 -----
  'security/detect-object-injection': 'off', // 允许动态属性访问

  // ----- Unicorn 规则 -----
  'unicorn/expiring-todo-comments': 'off', // 关闭 TODO 过期检查
  'unicorn/no-abusive-eslint-disable': 'off', // 允许全局禁用 ESLint
  'unicorn/filename-case': 'off', // 不强制文件名命名风格
  'unicorn/no-null': 'off', // 允许使用 null
  'unicorn/prevent-abbreviations': 'off', // 允许缩写变量名（如 props、ref）
  'unicorn/prefer-logical-operator-over-ternary': 'off', // 允许简单三元表达式
  'unicorn/prefer-spread': 'off', // 允许使用 Array.from、apply 等
  'unicorn/prefer-number-properties': 'off', // 允许全局 isNaN/isFinite
  'unicorn/consistent-function-scoping': 'off', // 允许嵌套函数定义
  'unicorn/no-invalid-remove-event-listener': 'off', // 关闭事件监听器参数检查
  'unicorn/prefer-add-event-listener': 'off', // 允许 onmessage/onerror 写法
  'unicorn/prefer-single-call': 'off', // 允许多次 push
  'unicorn/no-useless-switch-case': 'off', // 允许 fall-through case
  'unicorn/prefer-code-point': 'off', // 允许 fromCharCode/charCodeAt
  'unicorn/no-useless-spread': 'off', // 允许冗余的展开运算符
  'unicorn/prefer-blob-reading-methods': 'off', // 允许 FileReader API
  'unicorn/prefer-structured-clone': 'off', // 允许 JSON 深拷贝
  'unicorn/import-style': 'off', // 不强制特定 import 语法
  'unicorn/text-encoding-identifier-case': 'off', // 允许 utf-8 大小写
  'unicorn/no-object-as-default-parameter': 'off', // 允许对象字面量作为默认值
  'unicorn/prefer-ternary': 'off', // 允许 if-else 语句
  'unicorn/prefer-regexp-test': 'off', // 允许 String.match 检查匹配
  'unicorn/no-array-for-each': 'off', // 允许 Array.forEach

  // ----- ESLint 核心规则 -----
  'no-lone-blocks': 'off', // 允许独立代码块
  'no-void': 'off', // 允许 void 运算符
  'no-unused-vars': 'off', // 由 TypeScript 检查
  'no-prototype-builtins': 'off', // 允许直接调用 Object.prototype 方法
  'no-inner-declarations': 'off', // 允许块级函数声明
  'no-control-regex': 'off', // 允许正则中的控制字符
  'no-empty-character-class': 'off', // 允许空字符类
  'no-useless-escape': 'off', // 允许冗余转义（提高可读性）
  complexity: 'off', // 关闭圈复杂度限制
  'max-lines': 'off', // 关闭文件行数限制
  'no-console': 'off', // 允许 console（开发调试）
  'no-use-before-define': 'off', // 由 @typescript-eslint 接管
  'consistent-return': 'off', // 允许不一致的返回值
  'guard-for-in': 'off', // 现代代码推荐 for-of
  'no-bitwise': 'off', // 允许位运算（颜色处理等）
  'no-new': 'off', // 允许不存储结果的 new
  'no-unused-expressions': 'off', // 由 @typescript-eslint 接管
  'require-await': 'off', // 由 @typescript-eslint 接管
  'consistent-this': 'off', // 现代代码使用箭头函数
}
