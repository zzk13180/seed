<template>
  <div class="map-view-container">
    <el-card class="map-view-header">
      <template v-if="isControllerInitialized">
        <div>iconbar</div>
      </template>
    </el-card>
    <el-card class="map-view-content">
      <div class="map-view-canvas-container" ref="canvasContainerRef">
        <template v-if="isControllerInitialized">
          <MapGridCanvas />
        </template>
      </div>
    </el-card>
  </div>
</template>

<script setup lang="ts">
  import MapGridCanvas from './MapGridCanvas/MapGridCanvas.vue'
  import { useMapStore } from './map.store'

  const canvasContainerRef = ref<HTMLDivElement | null>(null)

  const { state, controller } = useMapStore()
  const isControllerInitialized = ref(false)

  onMounted(() => {
    console.log('MapView mounted', performance.now())
    controller.initialize(canvasContainerRef.value)
    isControllerInitialized.value = true
  })

  onBeforeUnmount(() => {
    controller.destroy()
  })
</script>

<style scoped lang="scss">
  .map-view-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    height: 100%;
    box-sizing: border-box;

    :deep(.el-card__body) {
      width: 100%;
      height: 100%;
    }

    .map-view-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .map-view-content {
      flex: 1;
      position: relative;

      :deep(.el-card__body) {
        padding: 0;
      }

      .map-view-canvas-container {
        width: 100%;
        height: 100%;
        position: absolute;
        left: 0;
        top: 0;

        > canvas {
          pointer-events: none;
        }
      }
    }
  }
</style>
