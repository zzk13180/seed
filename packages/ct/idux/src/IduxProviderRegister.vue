<!-- eslint-disable-next-line vue/valid-template-root -->
<template></template>

<script setup lang="ts">
  import { useDrawer } from '@idux/components/drawer'
  import { useMessage } from '@idux/components/message'
  import { useModal } from '@idux/components/modal'
  import { useNotification } from '@idux/components/notification'

  import { registerProviders } from '../index'

  const drawer = useDrawer()
  const notification = useNotification()
  const modal = useModal()
  const message = useMessage()

  registerProviders({ drawer, notification, modal, message })
</script>
