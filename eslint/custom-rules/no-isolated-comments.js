/**
 * 自定义规则：禁止孤立注释
 *
 * 规则说明：
 * 1. 注释必须紧邻代码（在代码上方或同一行），不允许孤立注释
 * 2. 例外：文件开头的注释块（从第1行开始的连续注释）允许存在
 *
 * 该规则支持自动修复，会删除孤立的注释行,
 * 主要是为了避免 eslint-plugin-sort-class-members 在排序类成员时会将注释行移到错误位置的问题。
 * 如果不需要此规则，可以将其移除。
 *
 * 注意：此规则仅应用于 TypeScript 文件（在 eslint.config.js 中配置 files: ['**\/*.ts']）
 *
 * @type {import('eslint').Rule.RuleModule}
 */
export const noIsolatedComments = {
  meta: {
    type: 'suggestion',
    docs: { description: '禁止孤立注释，注释必须紧邻代码或在文件开头' },
    fixable: 'code',
    schema: [],
    messages: { isolatedComment: '注释必须紧邻相关代码: "{{comment}}"' },
  },
  create(context) {
    function createFixer(fixer, sourceCode, comment) {
      const { line: startLine } = comment.loc.start
      const { line: endLine } = comment.loc.end
      const lineStart = sourceCode.getIndexFromLoc({ line: startLine, column: 0 })
      const nextLineStart =
        endLine < sourceCode.lines.length
          ? sourceCode.getIndexFromLoc({ line: endLine + 1, column: 0 })
          : sourceCode.text.length
      return fixer.removeRange([lineStart, nextLineStart])
    }

    function doesCommentBlockReachCode(startLineIndex, lines) {
      for (let i = startLineIndex; i < lines.length; i++) {
        const line = lines[i].trim()
        if (line === '') {
          return false
        }
        if (line.startsWith('//') || line.startsWith('/*') || line.startsWith('*')) {
          continue
        }
        return true
      }
      return false
    }

    return {
      Program() {
        const sourceCode = context.sourceCode || context.getSourceCode()
        const { lines } = sourceCode

        let fileHeaderEndLine = 0
        for (const [i, line] of lines.entries()) {
          const trimmed = line.trim()
          if (
            ['', '//', '/*', '*', '*/'].some(
              p => trimmed.startsWith(p) || trimmed.endsWith(p.slice(-2)),
            )
          ) {
            fileHeaderEndLine = i + 1
          } else {
            break
          }
        }

        for (const comment of sourceCode.getAllComments()) {
          if (comment.type !== 'Line') {
            continue
          }
          const commentLine = comment.loc.start.line
          if (commentLine <= fileHeaderEndLine) {
            continue
          }
          const lineContent = lines[commentLine - 1]
          if (lineContent.slice(0, comment.loc.start.column).trim().length > 0) {
            continue
          }
          if (doesCommentBlockReachCode(commentLine, lines)) {
            continue
          }

          context.report({
            loc: comment.loc,
            messageId: 'isolatedComment',
            data: {
              comment: `//${comment.value}`.slice(0, 50) + (comment.value.length > 48 ? '...' : ''),
            },
            fix: fixer => createFixer(fixer, sourceCode, comment),
          })
        }
      },
    }
  },
}
