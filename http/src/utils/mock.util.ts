import { resolve, extname } from 'path'
import { build } from 'esbuild'
import { sync } from 'fast-glob'
interface NodeModuleWithCompile extends NodeModule {
  _compile(code: string, filename: string): any
}
export enum RequestMethod {
  GET = 0,
  POST,
  PUT,
  DELETE,
  PATCH,
  ALL,
  OPTIONS,
  HEAD,
}
export interface MockConfig {
  url: string
  method?: 'get' | 'post'
  headers?: Record<string, string>
  response?: any
}
export async function loadMockConfig(): Promise<MockConfig[]> {
  const basePath = './data'
  const mockFiles = sync(`**/*.{ts,js}`, {
    cwd: basePath,
  })
  const mockConfigs: MockConfig[] = []
  for (let i = 0, len = mockFiles.length; i < len; i++) {
    const resolvePath = resolve(basePath, mockFiles[i])
    const result = await build({
      entryPoints: [resolvePath],
      outfile: 'out.js',
      write: false,
      platform: 'node',
      bundle: true,
      format: 'cjs',
      metafile: false,
    })
    const { text } = result.outputFiles[0]
    const item = await loadConfigFromBundledFile(resolvePath, text)
    mockConfigs.push(item)
  }
  return mockConfigs
}

async function loadConfigFromBundledFile(
  fileName: string,
  bundledCode: string,
): Promise<MockConfig> {
  const extension = extname(fileName)
  const defaultLoader = require.extensions[extension]!
  require.extensions[extension] = (module: NodeModule, filename: string) => {
    if (filename === fileName) {
      ;(module as NodeModuleWithCompile)._compile(bundledCode, filename)
    } else {
      defaultLoader(module, filename)
    }
  }
  delete require.cache[require.resolve(fileName)]
  const raw = require(fileName)
  const config = raw.__esModule && raw.default ? raw.default : raw
  require.extensions[extension] = defaultLoader
  return config
}

export function checkMockConfig(item: MockConfig): MockConfig[] {
  const result: MockConfig[] = []
  if (!item) return result
  if (!item.url) {
    for (const key in item) {
      // Reflect.has(item,'url')
      if (item[key].url) result.push(item[key])
    }
  }
  if (!result.length) result.push(item)
  return result
}
