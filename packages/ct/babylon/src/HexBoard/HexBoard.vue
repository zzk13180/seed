<template>
  <canvas ref="canvasRef"></canvas>
</template>

<script lang="ts" setup>
  import { ref, onMounted } from 'vue'
  import { HexBoard } from './HexBoard'

  const canvasRef = ref<HTMLCanvasElement>()

  onMounted(() => {
    const hexBoard = new HexBoard(canvasRef.value!)
    hexBoard.init()
  })
</script>

<style lang="less">
  canvas {
    width: 100%;
    height: 100%;
  }
</style>
