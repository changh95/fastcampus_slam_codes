#include <iostream>
#include <unordered_map>

int main() {
  std::unordered_map<std::string, double> umap = {
      {"One", 1}, {"Two", 2}, {"Three", 3}};

  umap["PI"] = 3.1415;
  umap["root2"] = 1.414;
  umap.insert(std::make_pair("e", 2.718));

  std::string key = "PI";
  if (umap.find(key) == umap.end()) {
    std::cout << key << " not found\n\n";
  } else {
    std::cout << "Found " << key << "\n\n";
  }

  key = "lambda";
  if (umap.find(key) == umap.end()) {
    std::cout << key << " not found\n\n";
  } else {
    std::cout << "Found " << key << "\n\n";
  }

  std::unordered_map<std::string, double>::iterator itr;
  std::cout << "\nAll Elements : \n";
  for (itr = umap.begin(); itr != umap.end(); itr++) {
    std::cout << itr->first << ", " << itr->second << std::endl;
  }
}