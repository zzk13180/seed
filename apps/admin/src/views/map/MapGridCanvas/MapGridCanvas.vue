<template>
  <canvas ref="gridCanvasRef"></canvas>
</template>

<script setup lang="ts">
  import { ref, onMounted, onUnmounted, inject } from 'vue'
  import { useMapStore } from '../map.store'
  import type { Ref } from 'vue'

  const { controller } = useMapStore()

  const gridCanvasRef = ref<HTMLCanvasElement | null>(null)
  const container = inject('container') as Ref<HTMLDivElement | null>

  onMounted(() => {
    controller.initializeGrid(gridCanvasRef.value, container.value)
  })

  onUnmounted(() => {
    controller.destroyGrid()
  })
</script>
