#include <iostream>
#include <vector>

int main() {
  std::vector<int> vec = {1, 2, 3, 4, 5};
  vec.push_back(6);

  for (const auto &elem : vec) {
    std::cout << elem << std::endl;
  }

  vec.clear();

  vec.push_back(7);
  vec.push_back(8);

  for (const auto &elem : vec) {
    std::cout << elem << std::endl;
  }

  vec.pop_back();

  for (const auto &elem : vec) {
    std::cout << elem << std::endl;
  }
}