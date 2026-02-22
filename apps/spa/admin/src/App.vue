<template>
  <el-config-provider :locale="zhCN">
    <router-view />
  </el-config-provider>
</template>

<script setup lang="ts">
  import zhCN from 'element-plus/es/locale/lang/zh-cn'
</script>
