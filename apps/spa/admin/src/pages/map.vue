<template>
  <MapView />
</template>

<script setup lang="ts">
  import { onMounted } from 'vue'
  import MapView from '@/views/map/MapView.vue'

  onMounted(() => {
    console.log('MapView component mounted')
  })
</script>
