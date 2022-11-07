import './styles/index.less'
import '@seed/vue-components/dist/style.css'
import { bootstrap } from '@seed/vue/main/bootstrap'
import { RootRoute, HomeRoute, TheDemoRoute } from './routes'

bootstrap([RootRoute, HomeRoute, TheDemoRoute])
