<template>
  <Counter />
  <SampleButton block @click="go('TheHome')">click go TheHome</SampleButton>
</template>

<script lang="ts" setup>
  import { Counter } from '@seed/vue/demo'
  import { SampleButton } from '@seed/vue-components'
  import { useGo } from '@seed/vue/hooks/useGo'

  const go = useGo()
</script>
