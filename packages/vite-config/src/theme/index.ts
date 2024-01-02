import globalVars from './globalLessVars/index.json'
import { colorPalette } from './colorPalette'

function genLessVars() {
  const modes: [string, string][] = [
    ['primary', '#1890ff'],
    ['info', '#1890ff'],
    ['pending', '#1890ff'],
    ['success', '#52c41a'],
    ['warning', '#faad14'],
    ['error', '#ff4d4f'],
  ]
  const primaryColorObj = new Map(modes.map(([key, value]) => [`@color-${key}`, value]))
  const arr: [string, number][] = [
    ['l50', -50],
    ['l40', -40],
    ['l30', -30],
    ['l20', -20],
    ['l10', -10],
    ['d10', 10],
    ['d20', 20],
    ['d30', 30],
    ['d40', 40],
  ]
  modes.forEach(([mode, color]) => {
    arr.forEach(([name, index]) => {
      primaryColorObj.set(`@color-${mode}-${name}`, colorPalette(color, index))
    })
  })

  const vars = { ...globalVars, ...Object.fromEntries(primaryColorObj) }

  return vars
}

export { colorPalette, genLessVars }
