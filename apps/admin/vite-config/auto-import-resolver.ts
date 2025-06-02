interface ElementPlusResolverOptions {
  /**
   * import style css or sass with components
   *
   * @default 'css'
   */
  importStyle?: boolean | 'css' | 'sass'

  /**
   * use commonjs lib & source css or scss for ssr
   */
  ssr?: boolean

  /**
   * specify element-plus version to load style
   *
   * @default installed version
   */
  version?: string

  /**
   * auto import for directives
   *
   * @default true
   */
  directives?: boolean

  /**
   * exclude component name, if match do not resolve the name
   */
  exclude?: RegExp

  /**
   * a list of component names that have no styles, so resolving their styles file should be prevented
   */
  noStylesComponents?: string[]

  /**
   * nightly version
   */
  nightly?: boolean
}

type ElementPlusResolverOptionsResolved = Required<Omit<ElementPlusResolverOptions, 'exclude'>> &
  Pick<ElementPlusResolverOptions, 'exclude'>

function kebabCase(key: string) {
  const result = key.replace(/([A-Z])/g, ' $1').trim()
  return result.split(' ').join('-').toLowerCase()
}

// eslint-disable-next-line consistent-return
function getSideEffects(dirName: string, options: ElementPlusResolverOptionsResolved): any {
  const { importStyle, ssr, nightly } = options
  // const themeFolder = '@element-plus/theme-chalk'
  const esComponentsFolder = '@element-plus/components'

  if (importStyle === 'sass') {
    console.log('custom-resolver importStyle 禁用 sass')
  } else if (importStyle === true || importStyle === 'css') {
    return [`${esComponentsFolder}/${dirName}/style/css`]
  }
}

function resolveComponent(name: string, options: ElementPlusResolverOptionsResolved): any {
  if (options.exclude && name.match(options.exclude)) {
    return
  }

  if (!name.match(/^El[A-Z]/)) {
    return
  }

  if (name.match(/^ElIcon.+/)) {
    // eslint-disable-next-line consistent-return
    return {
      name: name.replace(/^ElIcon/, ''),
      from: '@element-plus/icons-vue',
    }
  }

  const partialName = kebabCase(name.slice(2)) // ElTableColumn -> table-column

  // eslint-disable-next-line consistent-return
  return {
    name,
    from: '@element-plus/components',

    sideEffects: getSideEffects(partialName, options),
  }
}

function resolveDirective(name: string, options: ElementPlusResolverOptionsResolved): any {
  if (!options.directives) {
    return
  }

  const directives: Record<string, { importName: string; styleName: string }> = {
    Loading: { importName: 'ElLoadingDirective', styleName: 'loading' },
    Popover: { importName: 'ElPopoverDirective', styleName: 'popover' },
    InfiniteScroll: { importName: 'ElInfiniteScroll', styleName: 'infinite-scroll' },
  }

  const directive = directives[name]
  if (!directive) {
    return
  }

  // eslint-disable-next-line consistent-return
  return {
    name: directive.importName,
    from: '@element-plus/components',

    sideEffects: getSideEffects(directive.styleName, options),
  }
}

const noStylesComponents = ['ElAutoResizer']

/**
 * Resolver for Element Plus
 *
 * See https://github.com/antfu/vite-plugin-components/pull/28 for more details
 * See https://github.com/antfu/vite-plugin-components/issues/117 for more details
 *
 * @author @develar @nabaonan @sxzz
 * @link https://element-plus.org/ for element-plus
 *
 */
export function CustomElementPlusResolver(options: ElementPlusResolverOptions = {}): any[] {
  let optionsResolved: ElementPlusResolverOptionsResolved

  function resolveOptions() {
    if (optionsResolved) {
      return optionsResolved
    }
    optionsResolved = {
      ssr: false,
      version: '2.7.8',
      importStyle: 'css',
      directives: true,
      exclude: undefined,
      noStylesComponents: options.noStylesComponents || [],
      nightly: false,
      ...options,
    }
    return optionsResolved
  }

  return [
    {
      type: 'component',
      resolve: (name: string) => {
        const options = resolveOptions()

        if ([...options.noStylesComponents, ...noStylesComponents].includes(name)) {
          return resolveComponent(name, { ...options, importStyle: false })
        } else {
          return resolveComponent(name, options)
        }
      },
    },
    {
      type: 'directive',
      resolve: (name: string) => {
        return resolveDirective(name, resolveOptions())
      },
    },
  ]
}
