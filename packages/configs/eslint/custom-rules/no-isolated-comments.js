/**
 * 自定义规则：禁止类内部的孤立单行注释
 *
 * 背景问题：
 * eslint-plugin-sort-class-members 在对类成员进行自动排序时，不会将注释与其相关代码绑定在一起。
 * 如果一个单行注释（// comment）下方存在空行或其他非代码内容，排序后注释会留在原位，
 * 导致注释与其描述的代码分离，造成混乱。
 *
 * 解决方案：
 * 此规则检测类内部的「孤立单行注释」，即那些下方没有紧邻代码的单行注释。
 * 通过自动修复（删除孤立注释），确保 sort-class-members 排序时不会产生错位问题。
 *
 * 规则行为：
 * 1. 仅检查类（class）内部的注释，类外的注释不受影响
 * 2. 仅检查单行注释（// comment），多行注释（/* ... *\/）不受影响
 * 3. 跳过行尾注释（如：const x = 1 // 行尾注释）
 * 4. 如果注释下方紧邻代码（允许连续的注释块），则视为有效注释
 * 5. 如果注释下方是空行或直到类结束都没有代码，则报错并自动删除
 *
 * 示例（会报错）：
 * ```ts
 * class Example {
 *   // 这是孤立注释，下方是空行
 *
 *   someMethod() {}
 * }
 * ```
 *
 * 示例（正常通过）：
 * ```ts
 * class Example {
 *   // 这是有效注释，紧邻下方代码
 *   someMethod() {}
 * }
 * ```
 *
 * 配置位置：
 * 在 eslint.config.js 中，此规则仅应用于 TypeScript 文件（files: ['**\/*.ts']）
 *
 * @type {import('eslint').Rule.RuleModule}
 */
export const noIsolatedComments = {
  meta: {
    type: 'suggestion',
    docs: { description: '禁止类内部的孤立注释，注释必须紧邻代码' },
    fixable: 'code',
    schema: [],
    messages: { isolatedComment: '类内注释必须紧邻相关代码: "{{comment}}"' },
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

    function doesCommentBlockReachCode(startLineIndex, lines, classEndLine) {
      for (let i = startLineIndex; i < lines.length && i < classEndLine; i++) {
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

    function isCommentInRange(comment, startLine, endLine) {
      return comment.loc.start.line > startLine && comment.loc.end.line < endLine
    }

    return {
      ClassBody(node) {
        const sourceCode = context.sourceCode || context.getSourceCode()
        const { lines } = sourceCode
        const classStartLine = node.loc.start.line
        const classEndLine = node.loc.end.line

        for (const comment of sourceCode.getAllComments()) {
          // 只检查单行注释
          if (comment.type !== 'Line') {
            continue
          }
          // 只检查类内部的注释
          if (!isCommentInRange(comment, classStartLine, classEndLine)) {
            continue
          }
          const commentLine = comment.loc.start.line
          const lineContent = lines[commentLine - 1]
          // 跳过行尾注释（注释前有代码）
          if (lineContent.slice(0, comment.loc.start.column).trim().length > 0) {
            continue
          }
          // 检查注释是否紧邻下方代码
          if (doesCommentBlockReachCode(commentLine, lines, classEndLine)) {
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
