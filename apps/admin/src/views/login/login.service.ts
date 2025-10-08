import { ElMessage } from 'element-plus'
import type { MessageService } from './login.types'

/**
 * Element Plus 消息服务实现
 */
export class ElementMessageService implements MessageService {
  success(message: string): void {
    ElMessage.success(message)
  }

  error(message: string): void {
    ElMessage.error(message)
  }

  warning(message: string): void {
    ElMessage.warning(message)
  }

  info(message: string): void {
    ElMessage.info(message)
  }
}
