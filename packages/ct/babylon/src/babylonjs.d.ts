export {}
declare module '@babylonjs/core/Meshes/mesh' {
  interface Mesh {
    data?: any
  }
}
declare module '@babylonjs/core/Meshes/AbstractMesh' {
  interface AbstractMesh {
    data?: any
    parent?: any
  }
}
