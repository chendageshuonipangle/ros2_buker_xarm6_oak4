// Minimal STEP-to-STL converter for collision meshes using the installed Assimp library.
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: step_to_stl INPUT.step OUTPUT.stl\n";
    return 2;
  }
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(
      argv[1], aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                   aiProcess_GenSmoothNormals | aiProcess_SortByPType);
  if (scene == nullptr) {
    std::cerr << "STEP import failed: " << importer.GetErrorString() << "\n";
    return 1;
  }
  Assimp::Exporter exporter;
  const aiReturn result = exporter.Export(scene, "stlb", argv[2]);
  if (result != aiReturn_SUCCESS) {
    std::cerr << "STL export failed: " << exporter.GetErrorString() << "\n";
    return 1;
  }
  std::cout << "converted " << argv[1] << " -> " << argv[2] << "\n";
  return 0;
}
