<template>
  <div class="map-view-container">
    <el-card class="map-header">
      <div>iconbar</div>
    </el-card>
    <el-card class="map-content">
      <div class="map-canvas-container" ref="mapCanvasContainer">
        <MapGridCanvas />
      </div>
    </el-card>
  </div>
</template>

<script setup lang="ts">
  import { provide, ref } from 'vue'
  import MapGridCanvas from './MapGridCanvas/MapGridCanvas.vue'

  const mapCanvasContainer = ref<HTMLDivElement | null>(null)
  provide('container', mapCanvasContainer)
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

    .map-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .map-content {
      flex: 1;
      position: relative;

      :deep(.el-card__body) {
        padding: 0;
      }

      .map-canvas-container {
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
