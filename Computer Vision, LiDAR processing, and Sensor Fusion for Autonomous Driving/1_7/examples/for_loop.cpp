#include <iostream>
#include <map>
#include <vector>

int main() {
  std::vector<int> vec = {1, 2, 3, 4, 5};

  for (int i = 0; i < 5; i++) {
    std::cout << i << std::endl;
  }

  for (int i = 0; i < vec.size(); i++) {
    std::cout << vec[i] << std::endl;
  }

  for (const auto &i : vec) {
    std::cout << i << std::endl;
  }

  std::map<std::string, int> umap;
  umap["one"] = 1;
  umap["two"] = 2;
  umap["three"] = 3;
  umap["four"] = 4;
  umap["five"] = 5;

  for (const auto &[first, second] : umap) {
    std::cout << second << std::endl;
  }
}