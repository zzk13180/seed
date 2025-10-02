/**
 * WebSocket 事件系统的类型定义
 * 定义了客户端与服务端之间通信的消息格式和类型
 */

/**
 * 消息类型枚举
 * 定义所有可能的 WebSocket 消息类型
 */
export enum MessageType {
  /** 初始化连接消息 */
  INIT = 'init',
  /** 确认收到消息的回复 */
  ACK = 'ack',
  /** 机器人状态更新消息 */
  ROBOT_STATUS = 'robot_status',
  /** 机器人状态流转消息 */
  ROBOT_STATE_UPDATE = 'robot_state_update',
  /** 弹窗对话框消息 */
  DIALOG = 'dialog',
  /** 视频播放消息 */
  VIDEO = 'video',
}

/**
 * 机器人情绪类型
 * 表示机器人当前的情绪状态
 */
export type RobotMood = 'happy' | 'sad' | 'idle'

/**
 * 弹窗类型
 * 定义不同类型的提示弹窗
 */
export type DialogType = 'info' | 'warning' | 'error' | 'success'

/**
 * 机器人交互状态
 */
export type RobotState = 'idle' | 'listening' | 'thinking' | 'responding'

/**
 * 基础消息接口
 * 所有 WebSocket 消息的基础结构
 * @template T 消息类型
 * @template D 消息数据类型
 */
export interface BaseMessage<T extends MessageType, D = unknown> {
  /** 消息类型标识 */
  type: T
  /** 消息携带的数据 */
  data: D
}

/**
 * 初始化消息
 * 服务端向新连接的客户端发送的第一条消息
 */
export interface InitMessage extends BaseMessage<MessageType.INIT, { serverTime: number }> {}

/**
 * 确认消息
 * 服务端对客户端消息的确认回复
 */
export interface AckMessage extends BaseMessage<MessageType.ACK, string> {}

/**
 * 机器人状态消息
 * 定期推送机器人的当前状态信息
 */
export interface RobotStatusMessage
  extends BaseMessage<
    MessageType.ROBOT_STATUS,
    {
      /** 机器人当前情绪 */
      mood: RobotMood
      /** 机器人当前活动 */
      activity: string
    }
  > {}

/**
 * 机器人状态更新消息
 * 用于驱动前端进行状态转换
 */
export interface RobotStateUpdateMessage
  extends BaseMessage<
    MessageType.ROBOT_STATE_UPDATE,
    {
      /** 机器人当前状态 */
      state: RobotState
      /** 关联的表情/视频ID */
      expression?: string
      /** 关联的文本（如聆听到的或要回答的） */
      text?: string
    }
  > {}

/**
 * 弹窗对话框消息
 * 触发客户端显示提示弹窗
 */
export interface DialogMessage
  extends BaseMessage<
    MessageType.DIALOG,
    {
      /** 弹窗标题 */
      title: string
      /** 弹窗内容 */
      message: string
      /** 弹窗类型 */
      type: DialogType
      /** 显示时长（毫秒） */
      duration: number
    }
  > {}

/**
 * 视频播放消息
 * 指示客户端播放指定视频
 */
export interface VideoMessage
  extends BaseMessage<
    MessageType.VIDEO,
    {
      /** 视频文件路径 */
      videoPath: string
      /** 是否自动播放 */
      autoPlay: boolean
    }
  > {}

/**
 * 所有可能的出站消息类型联合
 * 服务端可以发送给客户端的所有消息类型
 */
export type OutgoingMessage =
  | InitMessage
  | AckMessage
  | RobotStatusMessage
  | RobotStateUpdateMessage
  | DialogMessage
  | VideoMessage
