<template>
  <PanelView />
</template>

<script setup lang="ts">
  import { onMounted } from 'vue'
  import PanelView from '@/views/panel/PanelView.vue'

  onMounted(() => {
    console.log('PanelView component mounted')
  })
</script>
