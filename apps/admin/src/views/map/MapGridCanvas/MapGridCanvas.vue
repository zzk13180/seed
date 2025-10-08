<template>
  <canvas ref="gridCanvasRef" class="map-grid-canvas"></canvas>
</template>

<script setup lang="ts">
  import { useMapStore } from '../map.store'

  const { controller } = useMapStore()

  const gridCanvasRef = ref<HTMLCanvasElement | null>(null)

  onMounted(() => {
    if (gridCanvasRef.value) {
      controller.initializeGrid(gridCanvasRef.value)
    }
  })

  onUnmounted(() => {
    controller.destroyGrid()
  })
</script>
