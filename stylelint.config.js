/** @type {import('stylelint').Config} */
export default {
  // 根配置文件标识
  root: true,

  // 插件
  plugins: ['stylelint-order', 'stylelint-scss'],

  // 语法解析器
  customSyntax: 'postcss-scss',

  // 继承配置
  extends: ['stylelint-config-standard'],

  // 忽略文件
  ignoreFiles: [
    '**/dist/',
    '**/public/',
    '**/node_modules/',
    '**/.svelte-kit/',
    'apps/docs/',
    '*.css',
    '**/*.js',
    '**/*.jsx',
    '**/*.tsx',
    '**/*.ts',
  ],

  // 规则配置
  rules: {
    // 允许规则
    'media-query-no-invalid': null,
    'declaration-block-no-redundant-longhand-properties': null,
    'keyframes-name-pattern': null,
    'function-no-unknown': null,
    'no-duplicate-selectors': null,
    'no-invalid-double-slash-comments': null,
    'selector-not-notation': null,
    'selector-class-pattern': null,
    'no-empty-source': null,
    'block-no-empty': null,
    'named-grid-areas-no-invalid': null,
    'no-descending-specificity': null,
    'font-family-no-missing-generic-family-keyword': null,

    // 伪类/伪元素
    'selector-pseudo-class-no-unknown': [true, { ignorePseudoClasses: ['global'] }],
    'selector-pseudo-element-no-unknown': [true, { ignorePseudoElements: ['v-deep'] }],

    // @ 规则
    'at-rule-no-unknown': [
      true,
      {
        ignoreAtRules: [
          'import',
          'tailwind',
          'apply',
          'variants',
          'responsive',
          'screen',
          'function',
          'if',
          'else',
          'warn',
          'each',
          'include',
          'mixin',
          'use',
        ],
      },
    ],

    // 导入表示法
    'import-notation': 'string',

    // 规则空行
    'rule-empty-line-before': ['always', { ignore: ['after-comment', 'first-nested'] }],

    // 单位
    'unit-no-unknown': [true, { ignoreUnits: ['rpx'] }],

    // CSS 属性排序
    'order/order': [
      ['dollar-variables', 'custom-properties', 'at-rules', 'declarations', { type: 'at-rule', name: 'supports' }, { type: 'at-rule', name: 'media' }, 'rules'],
      { severity: 'warning' },
    ],

    // 现代 CSS
    'color-function-notation': 'modern',
    'alpha-value-notation': 'percentage',
  },

  // Vue/HTML 文件覆盖配置
  overrides: [
    {
      files: ['*.vue', '**/*.vue', '*.html', '**/*.html'],
      extends: ['stylelint-config-standard', 'stylelint-config-recommended-vue'],
      customSyntax: 'postcss-html',
      rules: {
        'keyframes-name-pattern': null,
        'selector-pseudo-class-no-unknown': [true, { ignorePseudoClasses: ['deep', 'global'] }],
        'selector-pseudo-element-no-unknown': [true, { ignorePseudoElements: ['v-deep', 'v-global', 'v-slotted'] }],
        'color-function-notation': 'modern',
        'alpha-value-notation': 'percentage',
      },
    },
  ],
}
