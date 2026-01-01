/**
 * 类成员排序顺序（支持自动修复）
 *
 * 排序优先级：
 * 1. 静态成员优先于实例成员
 * 2. 按访问修饰符排序：public → protected → private
 * 3. 属性优先于方法
 * 4. 构造函数位于实例属性之后、实例方法之前
 *
 * 完整顺序：
 * ┌─────────────────────────────────────────────────────────┐
 * │ 1. 静态成员（按 public → protected → private 顺序）     │
 * │    - 静态属性                                           │
 * │    - 静态只读属性                                       │
 * │    - 静态 getter/setter                                 │
 * │    - 静态方法                                           │
 * ├─────────────────────────────────────────────────────────┤
 * │ 2. 实例属性（按 public → protected → private 顺序）     │
 * │    - 普通属性                                           │
 * │    - 带装饰器的属性（仅 public，如 @Column）            │
 * │    - 只读属性                                           │
 * ├─────────────────────────────────────────────────────────┤
 * │ 3. 构造函数                                             │
 * ├─────────────────────────────────────────────────────────┤
 * │ 4. 实例方法（按 public → protected → private 顺序）     │
 * │    - getter/setter                                      │
 * │    - 普通方法                                           │
 * └─────────────────────────────────────────────────────────┘
 */
export const classMembersOrder = [
  // ===== 1. 静态成员 =====
  ...['public', 'protected', 'private'].flatMap(access => [
    { type: 'property', static: true, accessibility: access },
    { type: 'property', static: true, accessibility: access, readonly: true },
    { type: 'method', static: true, accessibility: access, kind: 'get' },
    { type: 'method', static: true, accessibility: access, kind: 'set' },
    { type: 'method', static: true, accessibility: access },
  ]),

  // ===== 2. 实例属性 =====
  ...['public', 'protected', 'private'].flatMap(access => {
    const items = [{ type: 'property', static: false, accessibility: access }]
    // public 属性额外支持按装饰器分组（常用于 ORM 实体类如 TypeORM @Column）
    if (access === 'public') {
      items.push({ type: 'property', static: false, accessibility: access, groupByDecorator: true })
    }
    items.push({ type: 'property', static: false, accessibility: access, readonly: true })
    return items
  }),

  // ===== 3. 构造函数 =====
  'constructor',

  // ===== 4. 实例方法 =====
  ...['public', 'protected', 'private'].flatMap(access => [
    { type: 'method', static: false, accessibility: access, kind: 'get' },
    { type: 'method', static: false, accessibility: access, kind: 'set' },
    { type: 'method', static: false, accessibility: access },
  ]),
]
