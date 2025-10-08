# TypeORM Migrations

此目录存放数据库迁移文件。

## 使用方法

### 生成迁移
当你修改了 Entity 后，运行以下命令生成迁移文件：
```bash
pnpm migration:generate -- -n YourMigrationName
```

### 运行迁移
将待执行的迁移应用到数据库：
```bash
pnpm migration:run
```

### 回滚迁移
回滚最近一次迁移：
```bash
pnpm migration:revert
```

### 查看迁移状态
```bash
pnpm migration:show
```

## 注意事项

1. **生产环境**: 始终使用 Migration，禁止使用 `synchronize: true`
2. **命名规范**: 使用有意义的名称，如 `AddUserEmailColumn`、`CreateOrderTable`
3. **测试**: 在开发环境充分测试迁移脚本后再部署到生产环境
4. **备份**: 运行迁移前务必备份生产数据库
