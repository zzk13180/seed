<template>
  <div class="map-view-container">
    <el-card class="map-header">
      <div>iconbar</div>
    </el-card>
    <el-card class="map-content">
      <MapGrid />
    </el-card>
  </div>
</template>

<script setup lang="ts">
  import MapGrid from './MapGrid.vue'
</script>

<style scoped lang="scss">
  .map-view-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    height: 100%;
    box-sizing: border-box;

    .map-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .map-content {
      flex: 1;
      :deep(.el-card__body) {
        padding: 0;
      }
    }
  }
</style>
