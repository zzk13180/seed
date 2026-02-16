import { LANGUAGE_CODES as KNOWN_LANGUAGE_CODES } from './config'
import type { LanguageCode } from './config'

export function getLanguageFromURL(pathname: string): LanguageCode {
  const normalizedPath = pathname.toLowerCase()
  const langCode = normalizedPath.split('/').find(Boolean) ?? 'zh-cn'
  const matched = KNOWN_LANGUAGE_CODES.find(code => code === langCode)
  return matched ?? 'zh-cn'
}

export { LANGUAGES as KNOWN_LANGUAGES, LANGUAGE_CODES as KNOWN_LANGUAGE_CODES } from './config'
