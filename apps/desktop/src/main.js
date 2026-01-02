// @ts-check
/// <reference types="@tauri-apps/api" />

/**
 * Tauri 核心 API
 */
const { invoke } = globalThis.__TAURI__.core

/** @type {HTMLInputElement | null} */
let greetInputEl = null

/** @type {HTMLElement | null} */
let greetMsgEl = null

/**
 * 调用 Tauri 命令发送问候
 * @returns {Promise<void>}
 */
async function greet() {
  if (!greetInputEl || !greetMsgEl) {
    console.error('Required elements not found')
    return
  }

  try {
    // 了解更多 Tauri 命令: https://tauri.app/develop/calling-rust/
    const message = await invoke('greet', { name: greetInputEl.value })
    greetMsgEl.textContent = message
  } catch (error) {
    console.error('Failed to invoke greet command:', error)
    greetMsgEl.textContent = 'Error: Failed to greet'
  }
}

/**
 * 初始化应用
 */
function initialize() {
  greetInputEl = document.querySelector('#greet-input')
  greetMsgEl = document.querySelector('#greet-msg')

  const greetForm = document.querySelector('#greet-form')
  if (greetForm) {
    greetForm.addEventListener('submit', event => {
      event.preventDefault()
      greet()
    })
  }
}

// DOM 加载完成后初始化
globalThis.addEventListener('DOMContentLoaded', initialize)
