/**
 * 正则表达式最佳实践规则
 * 这些规则帮助编写高效、可读、安全的正则表达式
 */
export const regexpRules = {
  // ----- 错误预防 -----
  'regexp/no-dupe-disjunctions': 'error', // 禁止重复的分支模式
  'regexp/no-empty-alternative': 'error', // 禁止空的分支选择
  'regexp/no-empty-capturing-group': 'error', // 禁止空捕获组
  'regexp/no-empty-lookarounds-assertion': 'error', // 禁止空的环视断言
  'regexp/no-lazy-ends': 'error', // 禁止无意义的惰性量词结尾
  'regexp/no-obscure-range': 'error', // 禁止难以理解的字符范围
  'regexp/no-optional-assertion': 'error', // 禁止可选的断言
  'regexp/no-standalone-backslash': 'error', // 禁止孤立的反斜杠
  'regexp/no-super-linear-backtracking': 'error', // 防止灾难性回溯（ReDoS）
  'regexp/no-unused-capturing-group': 'error', // 禁止未使用的捕获组
  'regexp/no-zero-quantifier': 'error', // 禁止 {0} 量词
  'regexp/optimal-lookaround-quantifier': 'error', // 优化环视中的量词

  // ----- 代码风格（可自动修复）-----
  'regexp/match-any': 'error', // 规范 "匹配任意字符" 的写法
  'regexp/negation': 'error', // 规范否定字符类的写法
  'regexp/no-dupe-characters-character-class': 'error', // 禁止字符类中重复字符
  'regexp/no-trivially-nested-assertion': 'error', // 消除无意义的断言嵌套
  'regexp/no-trivially-nested-quantifier': 'error', // 消除无意义的量词嵌套
  'regexp/no-useless-character-class': 'error', // 消除无用的字符类
  'regexp/no-useless-flag': 'error', // 消除无意义的正则标志
  'regexp/no-useless-lazy': 'error', // 消除无意义的惰性量词
  'regexp/no-useless-range': 'error', // 消除多余的字符范围
  'regexp/prefer-d': ['error', { insideCharacterClass: 'ignore' }], // 优先使用 \d
  'regexp/prefer-plus-quantifier': 'error', // 优先使用 + 而非 {1,}
  'regexp/prefer-question-quantifier': 'error', // 优先使用 ? 而非 {0,1}
  'regexp/prefer-star-quantifier': 'error', // 优先使用 * 而非 {0,}
  'regexp/prefer-w': 'error', // 优先使用 \w
  'regexp/sort-alternatives': 'error', // 排序分支选项
  'regexp/sort-flags': 'error', // 排序正则标志（如 /gi → /gi）
  'regexp/strict': 'error', // 启用严格模式检查
}
