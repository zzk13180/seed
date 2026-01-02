/**
 * 判断字符串是否为有效的 Base64 格式
 * @param str 待检测字符串
 * @returns 是否为有效的 Base64 字符串
 */
function isBase64(str: string): boolean {
  if (typeof str !== 'string' || str.length === 0 || str.length % 4 !== 0) {
    return false
  }
  // 严格校验：= 只能出现在末尾，且最多2个
  return /^[A-Za-z0-9+/]+={0,2}$/.test(str) && !/=[^=]/.test(str)
}

/**
 * 编码字符串为 Base64
 *
 * @param str 要编码的字符串
 * @returns 编码后的 Base64 字符串
 */
export function encodeBase64(str: string): string {
  if (str === '') return ''

  try {
    const encoder = new TextEncoder()
    const bytes = encoder.encode(str)

    const chunkSize = 0x8000 // 32KB chunks
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
 *
 * @param base64 要解码的 Base64 字符串
 * @returns 解码后的字符串
 */
export function decodeBase64(base64: string): string {
  if (base64 === '') return ''

  if (!isBase64(base64)) {
    throw new Error('无效的 Base64 格式')
  }

  try {
    const binary = atob(base64)
    const bytes = new Uint8Array(binary.length)

    // 优化：直接赋值而不是逐个字符处理
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
