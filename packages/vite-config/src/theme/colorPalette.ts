import tinycolor2 from 'tinycolor2'

const hueStep = 2
const saturationStep = 0.16
const saturationStep2 = 0.05
const brightnessStep1 = 0.05
const brightnessStep2 = 0.15
const lightColorCount = 5
const darkColorCount = 4

function getHue(hsv: { h: number }, i: number, isLight: boolean): number {
  let hue: number
  if (hsv.h >= 60 && hsv.h <= 240) {
    hue = isLight ? hsv.h - hueStep * i : hsv.h + hueStep * i
  } else {
    hue = isLight ? hsv.h + hueStep * i : hsv.h - hueStep * i
  }
  if (hue < 0) {
    hue += 360
  } else if (hue >= 360) {
    hue -= 360
  }
  return Math.round(hue)
}

function getSaturation(
  hsv: { h: number; s: number },
  i: number,
  isLight: boolean,
): number {
  // grey color don't change saturation
  if (hsv.h === 0 && hsv.s === 0) {
    return hsv.s
  }
  let saturation: number
  if (isLight) {
    saturation = hsv.s - saturationStep * i
  } else if (i === darkColorCount) {
    saturation = hsv.s + saturationStep
  } else {
    saturation = hsv.s + saturationStep2 * i
  }
  if (saturation > 1) {
    saturation = 1
  }
  if (isLight && i === lightColorCount && saturation > 0.1) {
    saturation = 0.1
  }
  if (saturation < 0.06) {
    saturation = 0.06
  }
  return Number(saturation.toFixed(2))
}

function getValue(hsv: { v: number }, i: number, isLight: boolean): number {
  let value: number
  if (isLight) {
    value = hsv.v + brightnessStep1 * i
  } else {
    value = hsv.v - brightnessStep2 * i
  }
  if (value > 1) {
    value = 1
  }
  return Number(value.toFixed(2))
}

export function colorPalette(color: any, index: number) {
  const isLight = index <= 0
  const hsv = tinycolor2(color).toHsv()
  const i = Math.abs(index / 10)
  return tinycolor2({
    h: getHue(hsv, i, isLight),
    s: getSaturation(hsv, i, isLight),
    v: getValue(hsv, i, isLight),
  }).toHexString()
}
