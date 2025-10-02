<template>
  <div
    :class="[ns.b('dragger'), ns.is('dragover', dragover)]"
    @drop.prevent="onDrop"
    @dragover.prevent="onDragover"
    @dragleave.prevent="dragover = false"
  >
    <slot />
  </div>
</template>
<script lang="ts" setup>
  import { inject, ref } from 'vue'
  import { useNamespace } from '@seed/element-plus-hooks'
  import { useFormDisabled } from '@seed/element-plus-components/form'
  import { throwError } from '@seed/element-plus-utils/error'
  import { uploadContextKey } from './constants'
  import { uploadDraggerEmits, uploadDraggerProps } from './upload-dragger'

  const COMPONENT_NAME = 'ElUploadDrag'

  defineOptions({
    name: COMPONENT_NAME,
  })

  defineProps(uploadDraggerProps)
  const emit = defineEmits(uploadDraggerEmits)

  const uploaderContext = inject(uploadContextKey)
  if (!uploaderContext) {
    throwError(COMPONENT_NAME, 'usage: <el-upload><el-upload-dragger /></el-upload>')
  }

  const ns = useNamespace('upload')
  const dragover = ref(false)
  const disabled = useFormDisabled()

  const onDrop = (e: DragEvent) => {
    if (disabled.value) {
      return
    }
    dragover.value = false

    e.stopPropagation()

    const files = Array.from(e.dataTransfer!.files)
    emit('file', files)
  }

  const onDragover = () => {
    if (!disabled.value) {
      dragover.value = true
    }
  }
</script>
