<template>
  <li :class="[ns.b(), { [ns.e('center')]: center }]">
    <div :class="ns.e('tail')" />
    <div
      v-if="!$slots.dot"
      :class="defaultNodeKls"
      :style="{
        backgroundColor: color,
      }"
    >
      <el-icon v-if="icon" :class="ns.e('icon')">
        <component :is="icon" />
      </el-icon>
    </div>
    <div v-if="$slots.dot" :class="ns.e('dot')">
      <slot name="dot" />
    </div>

    <div :class="ns.e('wrapper')">
      <div v-if="!hideTimestamp && placement === 'top'" :class="[ns.e('timestamp'), ns.is('top')]">
        {{ timestamp }}
      </div>

      <div :class="ns.e('content')">
        <slot />
      </div>

      <div
        v-if="!hideTimestamp && placement === 'bottom'"
        :class="[ns.e('timestamp'), ns.is('bottom')]"
      >
        {{ timestamp }}
      </div>
    </div>
  </li>
</template>

<script lang="ts" setup>
  import { computed } from 'vue'
  import { ElIcon } from '@element-plus/components/icon'
  import { useNamespace } from '@element-plus/hooks'
  import { timelineItemProps } from './timeline-item'

  defineOptions({
    name: 'ElTimelineItem',
  })

  const props = defineProps(timelineItemProps)

  const ns = useNamespace('timeline-item')
  const defaultNodeKls = computed(() => [
    ns.e('node'),
    ns.em('node', props.size || ''),
    ns.em('node', props.type || ''),
    ns.is('hollow', props.hollow),
  ])
</script>
