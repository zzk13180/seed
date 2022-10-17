<template>
  <div class="container">
    <p>Welcome</p>
    <div class="row">
      <img src="/tauri.svg" class="logo tauri" alt="Tauri logo" />
      <img src="/javascript.svg" class="logo vanilla" alt="JavaScript logo" />
    </div>

    <div class="row">
      <div>
        <input v-model="name" placeholder="Enter a name..." />
        <button type="button" @click="greet">Greet</button>
      </div>
    </div>

    <p id="greet-msg">{{ msg }}</p>
    <div style="cursor: pointer" @click="go('HexBoard')">click go HexBoard</div>
  </div>
  <CodemirrorEditor />
</template>

<script lang="ts" setup>
  import { ref } from 'vue'
  import { CodemirrorEditor } from '@seed/ct/editors'
  import { useGo } from '@seed/common/hooks/useGo'
  import { invoke } from '@tauri-apps/api'

  const go = useGo()
  const name = ref('')
  const msg = ref('')

  function greet() {
    // tauri
    invoke<string>('greet', { name: name.value }).then((res) => {
      msg.value = res
    })
    invoke<boolean>('is_win_11').then((res) => console.log(res))
  }
</script>
