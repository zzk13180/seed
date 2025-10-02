import type { KNOWN_LANGUAGE_CODES } from './config'

export const langPathRegex = /\/([a-z]{2}_?[A-Z]{0,2})\//

export function getLanguageFromURL(pathname: string) {
  const langCodeMatch = pathname.match(langPathRegex)
  const langCode = langCodeMatch ? langCodeMatch[1] : 'zh_CN'
  return langCode as (typeof KNOWN_LANGUAGE_CODES)[number]
}

export { KNOWN_LANGUAGES, KNOWN_LANGUAGE_CODES } from './config'
