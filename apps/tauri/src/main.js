const { invoke } = globalThis.__TAURI__.core

let greetInputEl
let greetMsgEl

async function greet() {
  // Learn more about Tauri commands at https://tauri.app/develop/calling-rust/
  greetMsgEl.textContent = await invoke('greet', { name: greetInputEl.value })
}

globalThis.addEventListener('DOMContentLoaded', () => {
  greetInputEl = document.querySelector('#greet-input')
  greetMsgEl = document.querySelector('#greet-msg')
  document.querySelector('#greet-form').addEventListener('submit', e => {
    e.preventDefault()
    greet()
  })
})
