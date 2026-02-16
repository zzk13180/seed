/**
 * Astro 5.x Content Collections 配置
 *
 * Content Collections 提供：
 * - 类型安全的 frontmatter
 * - 自动生成的 TypeScript 类型
 * - 内置验证和错误提示
 * - 更好的内容查询 API
 */
import { defineCollection, z } from 'astro:content'
import { glob } from 'astro/loaders'

// 文档集合 Schema
const docsSchema = z.object({
  title: z.string(),
  description: z.string(),
  // 可选字段
  image: z
    .object({
      src: z.string(),
      alt: z.string(),
    })
    .optional(),
  draft: z.boolean().default(false),
  order: z.number().optional(), // 用于排序
  category: z.string().optional(),
  tags: z.array(z.string()).optional(),
  lastUpdated: z.date().optional(),
})

// 定义文档集合
const docs = defineCollection({
  loader: glob({ pattern: '**/*.{md,mdx}', base: './src/content/docs' }),
  schema: docsSchema,
})

// 博客集合（可选，未来扩展）
const blog = defineCollection({
  loader: glob({ pattern: '**/*.{md,mdx}', base: './src/content/blog' }),
  schema: z.object({
    title: z.string(),
    description: z.string(),
    pubDate: z.date(),
    author: z.string().default('Seed Team'),
    image: z
      .object({
        src: z.string(),
        alt: z.string(),
      })
      .optional(),
    tags: z.array(z.string()).default([]),
  }),
})

export const collections = {
  docs,
  blog,
}
