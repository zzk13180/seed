<template>
  <div class="map-grid-container" ref="containerRef">
    <canvas ref="gridCanvasRef"></canvas>
  </div>
</template>

<script setup lang="ts">
  import { ref, onMounted, onUnmounted } from 'vue'
  import { useMapStore } from './map.store'

  const mapStore = useMapStore()

  const containerRef = ref<HTMLDivElement | null>(null)
  const gridCanvasRef = ref<HTMLCanvasElement | null>(null)

  onMounted(() => {
    console.log('MapGrid mounted', containerRef.value)

    if (containerRef.value && gridCanvasRef.value) {
      mapStore.initializeMapView(containerRef.value)
      mapStore.initializeGrid(gridCanvasRef.value)
      mapStore.state.viewChangeCallback = () => mapStore.controller.drawGrid()
    }

    window.addEventListener('resize', mapStore.resizeGridCanvas)
    window.addEventListener('orientationchange', mapStore.resizeGridCanvas)
  })

  onUnmounted(() => {
    window.removeEventListener('resize', mapStore.resizeGridCanvas)
    window.removeEventListener('orientationchange', mapStore.resizeGridCanvas)

    mapStore.destroyMapView()
  })
</script>

<style scoped>
  .map-grid-container {
    width: 100%;
    height: 100%;
    position: relative;
  }

  canvas {
    width: 100%;
    height: 100%;
    display: block;
  }
</style>
