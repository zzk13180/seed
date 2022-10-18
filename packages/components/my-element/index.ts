import { LitElement, unsafeCSS, html } from 'lit'
import { customElement, property } from 'lit/decorators.js'
import litLogo from './assets/lit.svg'
import styles from './index.scss?inline'

@customElement('my-element')
export class MyElement extends LitElement {
  @property()
  docsHint = 'Lit'

  @property({ type: Number })
  count = 0

  override render() {
    return html`
      <div>
        <a href="https://lit.dev" target="_blank">
          <img src=${litLogo} class="logo lit" alt="Lit logo" />
        </a>
      </div>
      <slot></slot>
      <div class="card">
        <p @click=${this._onClick}> count is ${this.count} </p>
      </div>
      <p class="read-the-docs">${this.docsHint}</p>
    `
  }

  private _onClick() {
    this.count++
  }

  static override styles = unsafeCSS(styles)
}

declare global {
  interface HTMLElementTagNameMap {
    'my-element': MyElement
  }
}
