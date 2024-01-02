const fs = require('fs')
const path = require('path')
const less = require('less')

module.exports = async function (source) {
  const paletteLess = fs.readFileSync(source, 'utf8')
  const result = await new Promise((resolve, reject) => {
    less
      .parse(paletteLess, {
        paths: [path.dirname(source)],
      })
      .then(
        output => {
          const evalEnv = new less.contexts.Eval({
            javascriptEnabled: true,
          })
          const evaldRoot = output.eval(evalEnv)
          const ruleset = evaldRoot.rules
          const variables = ruleset.reduce((prev, current) => {
            if (current.variable === true) {
              const { name } = current
              const { value } = current
              prev[name] = value.toCSS()
            }
            return prev
          }, {})
          resolve(variables)
        },
        error => reject(error),
      )
  })
  return result
}
