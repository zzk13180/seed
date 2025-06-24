import type { Canvas } from 'fabric'

export function debugDownloadImageData(imageData: ImageData, filename: string) {
  try {
    const canvas = document.createElement('canvas')
    canvas.width = imageData.width
    canvas.height = imageData.height
    const ctx = canvas.getContext('2d')

    if (!ctx) {
      console.error('无法创建调试canvas上下文')
      return
    }

    ctx.putImageData(imageData, 0, 0)

    canvas.toBlob(blob => {
      if (!blob) {
        console.error('ImageData转blob失败')
        return
      }

      const url = URL.createObjectURL(blob)
      const link = document.createElement('a')
      link.href = url
      link.download = `${filename}.png`
      document.body.append(link)
      link.click()
      link.remove()
      URL.revokeObjectURL(url)

      console.log(`🔍 已下载调试图片: ${filename}.png`)
    }, 'image/png')
  } catch (error) {
    console.error('下载ImageData调试图片失败:', error)
  }
}

export function debugDownloadCanvas(canvas: HTMLCanvasElement, filename: string): void {
  try {
    canvas.toBlob(blob => {
      if (!blob) {
        console.error('Canvas转blob失败')
        return
      }

      const url = URL.createObjectURL(blob)
      const link = document.createElement('a')
      link.href = url
      link.download = `${filename}.png`
      document.body.append(link)
      link.click()
      link.remove()
      URL.revokeObjectURL(url)

      console.log(`🔍 已下载调试图片: ${filename}.png`)
    }, 'image/png')
  } catch (error) {
    console.error('下载Canvas调试图片失败:', error)
  }
}

export function debugDownloadFabricCanvas(canvas: Canvas, filename: string): void {
  try {
    // @ts-ignore
    const dataUrl = canvas.toDataURL({ format: 'png' })

    fetch(dataUrl)
      .then(res => res.blob())
      .then(blob => {
        const url = URL.createObjectURL(blob)
        const link = document.createElement('a')
        link.href = url
        link.download = `${filename}.png`
        document.body.append(link)
        link.click()
        link.remove()
        URL.revokeObjectURL(url)

        console.log(`🔍 已下载Fabric Canvas调试图片: ${filename}.png`)
      })
      .catch(error => {
        console.error('下载Fabric Canvas调试图片失败:', error)
      })
  } catch (error) {
    console.error('下载Fabric Canvas调试图片失败:', error)
  }
}

export function debugCanvasState(canvas: Canvas): void {
  const canvasElement = canvas.getElement()
  const container = canvasElement.parentElement

  console.log('🔍 完整Canvas状态调试:', {
    fabric: {
      width: canvas.getWidth(),
      height: canvas.getHeight(),
      zoom: canvas.getZoom(),
      objectsCount: canvas.getObjects().length,
      backgroundColor: canvas.backgroundColor,
    },
    dom: {
      width: canvasElement.width,
      height: canvasElement.height,
      clientWidth: canvasElement.clientWidth,
      clientHeight: canvasElement.clientHeight,
      offsetWidth: canvasElement.offsetWidth,
      offsetHeight: canvasElement.offsetHeight,
      scrollWidth: canvasElement.scrollWidth,
      scrollHeight: canvasElement.scrollHeight,
    },
    style: {
      width: canvasElement.style.width,
      height: canvasElement.style.height,
      display: canvasElement.style.display,
      visibility: canvasElement.style.visibility,
      position: canvasElement.style.position,
      zIndex: canvasElement.style.zIndex,
    },
    computed: globalThis.getComputedStyle(canvasElement),
    container: container
      ? {
          clientWidth: container.clientWidth,
          clientHeight: container.clientHeight,
          scrollWidth: container.scrollWidth,
          scrollHeight: container.scrollHeight,
          style: globalThis.getComputedStyle(container),
        }
      : null,
    viewport: {
      innerWidth: window.innerWidth,
      innerHeight: window.innerHeight,
      devicePixelRatio: window.devicePixelRatio,
    },
  })
}

export const testRender = () => {
  // 创建测试图像数据
  const testWidth = 100
  const testHeight = 100
  const testImageData = new ImageData(testWidth, testHeight)

  // 填充测试图案
  for (let y = 0; y < testHeight; y++) {
    for (let x = 0; x < testWidth; x++) {
      const index = (y * testWidth + x) * 4
      // 创建棋盘图案
      const isWhite = (Math.floor(x / 10) + Math.floor(y / 10)) % 2 === 0
      const value = isWhite ? 255 : 0

      testImageData.data[index] = value // R
      testImageData.data[index + 1] = value // G
      testImageData.data[index + 2] = value // B
      testImageData.data[index + 3] = 255 // A
    }
  }

  console.log('🧪 开始测试渲染棋盘图案')
  return testImageData
}
