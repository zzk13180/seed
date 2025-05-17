<template>
  <focus-group-collection>
    <roving-focus-group-impl v-bind="$attrs">
      <slot />
    </roving-focus-group-impl>
  </focus-group-collection>
</template>

<script lang="ts">
  import { defineComponent } from 'vue'
  import RovingFocusGroupImpl from './RovingFocusGroupImpl.vue'
  import { Collection as FocusGroupCollection } from './roving-focus-group'

  export default defineComponent({
    name: 'RovingFocusGroup',
    components: {
      FocusGroupCollection,
      RovingFocusGroupImpl,
    },
  })
</script>
