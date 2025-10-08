declare module 'diff-match-patch' {
  export class diff_match_patch {
    diff_main(text1: string, text2: string): Array<[number, string]>
    diff_cleanupSemantic(diffs: Array<[number, string]>): void
    diff_cleanupEfficiency(diffs: Array<[number, string]>): void
    diff_prettyHtml(diffs: Array<[number, string]>): string
    match_main(text: string, pattern: string, loc: number): number
    patch_make(
      text1: string,
      text2?: string | Array<[number, string]>,
      diffs?: Array<[number, string]>,
    ): Array<patch_obj>
    patch_apply(patches: Array<patch_obj>, text: string): [string, boolean[]]
    patch_toText(patches: Array<patch_obj>): string
    patch_fromText(text: string): Array<patch_obj>
  }

  export class patch_obj {
    diffs: Array<[number, string]>
    start1: number
    start2: number
    length1: number
    length2: number
  }

  export const DIFF_DELETE: -1
  export const DIFF_INSERT: 1
  export const DIFF_EQUAL: 0
}
