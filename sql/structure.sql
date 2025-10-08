# ========================================
# 数据库初始化脚本
# ========================================

-- 创建用户表
DROP TABLE IF EXISTS t_user;
CREATE TABLE t_user (
  id BIGINT AUTO_INCREMENT PRIMARY KEY COMMENT '主键ID',
  username VARCHAR(50) NOT NULL COMMENT '用户名',
  password VARCHAR(255) NOT NULL COMMENT '密码',
  nickname VARCHAR(50) DEFAULT NULL COMMENT '昵称',
  email VARCHAR(100) DEFAULT NULL COMMENT '邮箱',
  phone VARCHAR(20) DEFAULT NULL COMMENT '手机号',
  avatar VARCHAR(255) DEFAULT NULL COMMENT '头像URL',
  status INT NOT NULL DEFAULT 1 COMMENT '状态: 0-禁用 1-启用',
  deleted BIT NOT NULL DEFAULT 0 COMMENT '是否删除: 0-未删除 1-已删除',
  created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '创建时间',
  created_by BIGINT DEFAULT NULL COMMENT '创建人ID',
  updated_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP COMMENT '更新时间',
  updated_by BIGINT DEFAULT NULL COMMENT '更新人ID',
  UNIQUE KEY uk_username (username),
  UNIQUE KEY uk_email (email),
  INDEX idx_username (username),
  INDEX idx_email (email),
  INDEX idx_phone (phone),
  INDEX idx_status (status)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci COMMENT='用户表';

-- 创建角色表 (预留)
DROP TABLE IF EXISTS t_role;
CREATE TABLE t_role (
  id BIGINT AUTO_INCREMENT PRIMARY KEY COMMENT '主键ID',
  code VARCHAR(50) NOT NULL COMMENT '角色编码',
  name VARCHAR(100) NOT NULL COMMENT '角色名称',
  description VARCHAR(255) DEFAULT NULL COMMENT '角色描述',
  status INT NOT NULL DEFAULT 1 COMMENT '状态: 0-禁用 1-启用',
  deleted BIT NOT NULL DEFAULT 0 COMMENT '是否删除',
  created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '创建时间',
  created_by BIGINT DEFAULT NULL COMMENT '创建人ID',
  updated_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP COMMENT '更新时间',
  updated_by BIGINT DEFAULT NULL COMMENT '更新人ID',
  UNIQUE KEY uk_code (code),
  INDEX idx_code (code)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci COMMENT='角色表';

-- 创建用户角色关联表 (预留)
DROP TABLE IF EXISTS t_user_role;
CREATE TABLE t_user_role (
  id BIGINT AUTO_INCREMENT PRIMARY KEY COMMENT '主键ID',
  user_id BIGINT NOT NULL COMMENT '用户ID',
  role_id BIGINT NOT NULL COMMENT '角色ID',
  created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '创建时间',
  UNIQUE KEY uk_user_role (user_id, role_id),
  INDEX idx_user_id (user_id),
  INDEX idx_role_id (role_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci COMMENT='用户角色关联表';
