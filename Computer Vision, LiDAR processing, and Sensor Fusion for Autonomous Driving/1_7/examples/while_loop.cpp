#include <iostream>
#include <map>
#include <vector>

int main() {
  std::vector<int> vec = {1, 2, 3, 4, 5};

  int i = 0;
  while (i < 5) {
    std::cout << vec[i] << std::endl;
    i++;
  }
}