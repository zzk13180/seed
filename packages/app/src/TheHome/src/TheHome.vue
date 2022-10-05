<template>
  <div class="container">
    <h1>Welcome to Tauri!</h1>

    <div class="row">
      <a href="https://tauri.app" target="_blank">
        <img src="/tauri.svg" class="logo tauri" alt="Tauri logo" />
      </a>
      <a href="https://developer.mozilla.org/en-US/docs/Web/JavaScript" target="_blank">
        <img src="/javascript.svg" class="logo vanilla" alt="JavaScript logo" />
      </a>
    </div>

    <p>Click on the Tauri logo to learn more about the framework</p>

    <div class="row">
      <div>
        <input v-model="name" placeholder="Enter a name..." />
        <button type="button" @click="greet">Greet</button>
      </div>
    </div>

    <p id="greet-msg">{{ msg }}</p>
  </div>
  <CodemirrorEditor />
</template>

<script lang="ts" setup>
  import { ref } from 'vue'
  import { CodemirrorEditor } from '@seed/ct/editors'
  import { invoke } from '@tauri-apps/api'

  const name = ref('')
  const msg = ref('')

  function greet() {
    invoke<string>('greet', { name: name.value }).then((res) => {
      msg.value = res
    })
    invoke<boolean>('is_win_11').then((res) => console.log(res))
  }
</script>
