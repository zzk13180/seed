<template>
  <div class="container">
    <p className="text-3xl font-bold underline">Welcome</p>
    <div class="row">
      <custom-element />
      <img src="/tauri.svg" class="logo tauri" alt="Tauri logo" />
      <img class="logo vanill" src="/javascript.svg" alt="JavaScript logo" />
    </div>

    <div class="row">
      <div>
        <input v-model="name" placeholder="Enter a name..." />
        <SampleButton @click="greet">Greet</SampleButton>
      </div>
    </div>

    <p id="greet-msg">{{ msg }}</p>
    <SampleButton mode="default" @click="go('TheDemo')">click go TheDemo</SampleButton>
  </div>
</template>

<script lang="ts" setup>
  import { ref } from 'vue'
  import { useGo } from '@seed/vue/hooks/useGo'
  import { invoke } from '@tauri-apps/api'

  import { SampleButton } from '@seed/vue-components'
  import '@seed/custom-element'

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
