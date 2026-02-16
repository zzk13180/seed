/**
 * 判断字符串是否为有效的 Base64 格式
 */
function isBase64(str: string): boolean {
  if (typeof str !== 'string' || str.length === 0 || str.length % 4 !== 0) {
    return false
  }

  if (!/^[A-Za-z0-9+/]+={0,2}$/.test(str)) {
    return false
  }

  const firstPaddingIndex = str.indexOf('=')
  if (firstPaddingIndex === -1) {
    return true
  }

  return str
    .slice(firstPaddingIndex)
    .split('')
    .every(char => char === '=')
}

/**
 * 编码字符串为 Base64
 */
export function encodeBase64(str: string): string {
  if (str === '') return ''

  try {
    const encoder = new TextEncoder()
    const bytes = encoder.encode(str)

    const chunkSize = 0x80_00 // 32KB chunks
    let binary = ''

    for (let i = 0; i < bytes.length; i += chunkSize) {
      const chunk = bytes.subarray(i, Math.min(i + chunkSize, bytes.length))
      binary += String.fromCharCode(...chunk)
    }

    return btoa(binary)
  } catch (error) {
    console.warn('编码字符串失败:', error)
    throw new Error('Base64 编码失败')
  }
}

/**
 * 解码 Base64 字符串
 */
export function decodeBase64(base64: string): string {
  if (base64 === '') return ''

  if (!isBase64(base64)) {
    throw new Error('无效的 Base64 格式')
  }

  try {
    const binary = atob(base64)
    const bytes = new Uint8Array(binary.length)

    for (let i = 0; i < binary.length; i++) {
      bytes[i] = binary.charCodeAt(i)
    }

    const decoder = new TextDecoder()
    return decoder.decode(bytes)
  } catch (error) {
    console.warn('解码字符串失败:', error)
    throw new Error('Base64 解码失败')
  }
}
