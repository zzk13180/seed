import { PrimaryGeneratedColumn, CreateDateColumn, UpdateDateColumn, Column } from 'typeorm'

/**
 * 基础实体类
 * 包含通用字段：id、创建时间、更新时间、创建人、更新人、删除标记
 */
export abstract class BaseEntity {
  @PrimaryGeneratedColumn({ comment: '主键ID' })
  id!: number

  @CreateDateColumn({
    name: 'created_at',
    type: 'datetime',
    comment: '创建时间',
  })
  createdAt!: Date

  @Column({
    name: 'created_by',
    type: 'bigint',
    nullable: true,
    comment: '创建人ID',
  })
  createdBy!: number | null

  @UpdateDateColumn({
    name: 'updated_at',
    type: 'datetime',
    comment: '更新时间',
  })
  updatedAt!: Date

  @Column({
    name: 'updated_by',
    type: 'bigint',
    nullable: true,
    comment: '更新人ID',
  })
  updatedBy!: number | null

  @Column({
    name: 'deleted',
    type: 'bit',
    default: 0,
    comment: '是否删除: 0-未删除 1-已删除',
  })
  deleted!: boolean
}
