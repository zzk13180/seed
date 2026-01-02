-- ========================================
-- 数据库初始化脚本 (PostgreSQL)
-- ========================================

-- 创建用户表
DROP TABLE IF EXISTS t_user CASCADE;
CREATE TABLE t_user (
  id BIGSERIAL PRIMARY KEY,
  username VARCHAR(50) NOT NULL UNIQUE,
  password VARCHAR(255) NOT NULL,
  nickname VARCHAR(50) DEFAULT NULL,
  email VARCHAR(100) DEFAULT NULL UNIQUE,
  phone VARCHAR(20) DEFAULT NULL,
  avatar VARCHAR(255) DEFAULT NULL,
  status INTEGER NOT NULL DEFAULT 1,
  deleted INTEGER NOT NULL DEFAULT 0,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  created_by BIGINT DEFAULT NULL,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_by BIGINT DEFAULT NULL
);

-- 创建索引
CREATE INDEX idx_username ON t_user(username);
CREATE INDEX idx_email ON t_user(email);
CREATE INDEX idx_phone ON t_user(phone);
CREATE INDEX idx_status ON t_user(status);

COMMENT ON TABLE t_user IS '用户表';
COMMENT ON COLUMN t_user.id IS '主键ID';
COMMENT ON COLUMN t_user.username IS '用户名';
COMMENT ON COLUMN t_user.password IS '密码';
COMMENT ON COLUMN t_user.nickname IS '昵称';
COMMENT ON COLUMN t_user.email IS '邮箱';
COMMENT ON COLUMN t_user.phone IS '手机号';
COMMENT ON COLUMN t_user.avatar IS '头像URL';
COMMENT ON COLUMN t_user.status IS '状态: 0-禁用 1-启用';
COMMENT ON COLUMN t_user.deleted IS '是否删除: 0-未删除 1-已删除';
COMMENT ON COLUMN t_user.created_at IS '创建时间';
COMMENT ON COLUMN t_user.created_by IS '创建人ID';
COMMENT ON COLUMN t_user.updated_at IS '更新时间';
COMMENT ON COLUMN t_user.updated_by IS '更新人ID';

-- 创建角色表 (预留)
DROP TABLE IF EXISTS t_role CASCADE;
CREATE TABLE t_role (
  id BIGSERIAL PRIMARY KEY,
  code VARCHAR(50) NOT NULL UNIQUE,
  name VARCHAR(100) NOT NULL,
  description VARCHAR(255) DEFAULT NULL,
  status INTEGER NOT NULL DEFAULT 1,
  deleted INTEGER NOT NULL DEFAULT 0,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  created_by BIGINT DEFAULT NULL,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_by BIGINT DEFAULT NULL
);

CREATE INDEX idx_role_code ON t_role(code);

COMMENT ON TABLE t_role IS '角色表';

-- 创建用户角色关联表 (预留)
DROP TABLE IF EXISTS t_user_role CASCADE;
CREATE TABLE t_user_role (
  id BIGSERIAL PRIMARY KEY,
  user_id BIGINT NOT NULL,
  role_id BIGINT NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(user_id, role_id)
);

CREATE INDEX idx_user_role_user_id ON t_user_role(user_id);
CREATE INDEX idx_user_role_role_id ON t_user_role(role_id);

COMMENT ON TABLE t_user_role IS '用户角色关联表';
