<template>
  <div class="container">
    <p>Welcome</p>
    <my-element>
      <h1>Lit</h1>
    </my-element>
    <div class="row">
      <img src="/tauri.svg" class="logo tauri" alt="Tauri logo" />
      <img class="w-16 md:w-32 lg:w-48" src="/javascript.svg" alt="JavaScript logo" />
    </div>

    <div class="row">
      <div>
        <input v-model="name" placeholder="Enter a name..." />
        <button type="button" @click="greet">Greet</button>
      </div>
    </div>

    <p id="greet-msg">{{ msg }}</p>
    <div @click="go('TheDemo')">click go TheDemo</div>
  </div>
</template>

<script lang="ts" setup>
  import { ref } from 'vue'
  import { useGo } from '@seed/vue/hooks/useGo'
  import { invoke } from '@tauri-apps/api'
  import '@seed/my-element'

  const go = useGo()
  const name = ref('')
  const msg = ref('')

  function greet() {
    // tauri
    invoke<string>('greet', { name: name.value }).then(res => {
      msg.value = res
    })
    invoke<boolean>('is_win_11').then(res => console.log(res))
  }
</script>
