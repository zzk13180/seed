import fs from 'fs'

export function fileExists(f: string) {
  try {
    fs.accessSync(f, fs.constants.W_OK)
    return true
  } catch (error) {
    return false
  }
}
