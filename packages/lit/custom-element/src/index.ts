import { LitElement, unsafeCSS, html } from 'lit'
import { customElement, property } from 'lit/decorators.js'
import litLogo from '../assets/lit.svg'
import styles from './index.scss?inline'

@customElement('custom-element')
export class CustomElement extends LitElement {
  @property()
  docsHint = 'Lit'

  @property({ type: Number })
  count = 0

  override render() {
    return html`<img src=${litLogo} class="logo lit" alt="Lit logo" />`
  }

  static override styles = unsafeCSS(styles)
}

declare global {
  interface HTMLElementTagNameMap {
    'custom-element': CustomElement
  }
}
