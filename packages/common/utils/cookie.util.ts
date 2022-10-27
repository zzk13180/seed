import { isNumber, isString } from './is.util'

function cookieWrite(
  name: string,
  value?: string,
  expires?: number | string,
  doc = document,
  path?: string,
  domain?: string,
  secure = false,
) {
  const cookie: string[] = []
  cookie.push(`${name}=${encodeURIComponent(value || '')}`)
  if (isNumber(expires)) {
    cookie.push(`expires=${new Date(expires).toUTCString()}`)
  }
  if (isString(path)) {
    cookie.push(`path=${path}`)
  }
  if (isString(domain)) {
    cookie.push(`domain=${domain}`)
  }
  if (secure === true) {
    cookie.push('secure')
  }
  doc.cookie = cookie.join('; ')
}

function cookieRead(name: string, doc = document) {
  if (!doc) {
    return null
  }
  const match = doc.cookie.match(new RegExp(`(^|;\\s*)(${name})=([^;]*)`))
  return match ? decodeURIComponent(match[3]) : null
}

function cookieRemove(name: string, doc = document) {
  cookieWrite(name, '', Date.now() - 86400000, doc)
}

export { cookieWrite, cookieRead, cookieRemove }
