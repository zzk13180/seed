<template>
  <div>2233</div>
</template>

<script setup lang="ts">
  import { useRoute, useRouter } from 'vue-router'

  const { params, query } = useRoute()
  const { path } = params

  console.log('RedirectView', path, query)

  const router = useRouter()
  router.replace({ path: `/${path}`, query })
</script>
