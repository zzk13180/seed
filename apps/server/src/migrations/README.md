# Drizzle Migrations

此目录存放数据库迁移文件。

## 使用方法

### 生成迁移

当你修改了 Schema 后，运行以下命令生成迁移文件：

```bash
pnpm db:generate
```

### 运行迁移

将待执行的迁移应用到数据库：

```bash
pnpm db:migrate
```

### 推送 Schema（开发用）

直接将 Schema 推送到数据库，不生成迁移文件（仅用于开发）：

```bash
pnpm db:push
```

### 查看数据库 Studio

启动 Drizzle Studio 可视化管理数据库：

```bash
pnpm db:studio
```

## 注意事项

1. **生产环境**: 始终使用 Migration，禁止使用 `synchronize: true`
2. **命名规范**: 使用有意义的名称，如 `AddUserEmailColumn`、`CreateOrderTable`
3. **测试**: 在开发环境充分测试迁移脚本后再部署到生产环境
4. **备份**: 运行迁移前务必备份生产数据库
