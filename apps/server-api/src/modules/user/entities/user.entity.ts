import { Entity, Column, Index } from 'typeorm'
import { BaseEntity } from '../../../common/entities/base.entity'
import { UserStatus } from '../../../common/enums/user.enum'

/**
 * 用户实体
 */
@Entity('t_user')
export class User extends BaseEntity {
  @Column({
    type: 'varchar',
    length: 50,
    unique: true,
    comment: '用户名',
  })
  @Index()
  username!: string

  @Column({
    type: 'varchar',
    length: 255,
    comment: '密码',
  })
  password!: string

  @Column({
    type: 'varchar',
    length: 50,
    nullable: true,
    comment: '昵称',
  })
  nickname!: string | null

  @Column({
    type: 'varchar',
    length: 100,
    unique: true,
    nullable: true,
    comment: '邮箱',
  })
  @Index()
  email!: string | null

  @Column({
    type: 'varchar',
    length: 20,
    nullable: true,
    comment: '手机号',
  })
  @Index()
  phone!: string | null

  @Column({
    type: 'varchar',
    length: 255,
    nullable: true,
    comment: '头像URL',
  })
  avatar!: string | null

  @Column({
    type: 'int',
    default: UserStatus.ENABLED,
    comment: '状态: 0-禁用 1-启用',
  })
  @Index()
  status!: UserStatus
}
