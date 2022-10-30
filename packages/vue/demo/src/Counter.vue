<template>
  <div class="counter">
    <button @click="subtract()">-</button>
    <pre>{{ count }}</pre>
    <button @click="add()">+</button>
  </div>
</template>

<script lang="ts" setup>
  import { ref } from 'vue'

  const count = ref(0)
  const add = () => (count.value += 1)
  const subtract = () => (count.value -= 1)
</script>

<style>
  .counter {
    display: grid;
    font-size: 2em;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    margin-top: 2em;
    place-items: center;
  }
</style>
