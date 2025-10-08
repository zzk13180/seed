import { diff_match_patch as DiffMatchPatch } from 'diff-match-patch'
import type { ChatItem } from './typewriter.util'

export function diff(a: ChatItem | undefined, b: ChatItem): [number, string][] {
  if (a?.id === b.id && a.role === b.role) {
    return calculatePatch(a, b)
  }
  return [[1, b.text]]
}

function calculatePatch(a: ChatItem, b: ChatItem): [number, string][] {
  const differ = new DiffMatchPatch()
  const delta = differ.diff_main(a.text, b.text)
  return delta
}
