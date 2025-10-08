-- ========================================
-- 初始化数据 (PostgreSQL)
-- ========================================

-- 插入管理员用户 (密码: admin123)
INSERT INTO t_user (username, password, nickname, email, status) VALUES
('admin', '$2b$10$YourHashedPasswordHere', '管理员', 'admin@example.com', 1);

-- 插入测试用户 (密码: test123)
INSERT INTO t_user (username, password, nickname, email, status) VALUES
('test', '$2b$10$YourHashedPasswordHere', '测试用户', 'test@example.com', 1);

-- 插入角色
INSERT INTO t_role (code, name, description, status) VALUES
('ADMIN', '管理员', '系统管理员角色', 1),
('USER', '普通用户', '普通用户角色', 1);
