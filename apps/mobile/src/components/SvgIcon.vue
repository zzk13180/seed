<template>
  <svg aria-hidden="true">
    <use :xlink:href="symbolId" />
  </svg>
</template>

<script setup>
  import { computed } from 'vue'

  const props = defineProps({
    name: {
      type: String,
      required: true,
    },
    prefix: {
      type: String,
      default: 'icon',
    },
  })

  const symbolId = computed(() => `#${props.prefix}-${props.name}`)
</script>
