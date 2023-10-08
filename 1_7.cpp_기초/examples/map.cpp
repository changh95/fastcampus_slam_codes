#include <iostream>
#include <map>
#include <string>

void print_map(const std::string &comment,
               const std::map<std::string, int> &m) {
  std::cout << comment;
  for (const auto &[key, value] : m)
    std::cout << "[" << key << "] = " << value << "; ";
  std::cout << "\n";
}

int main() {
  std::map<std::string, int> m{{"Autonomous Driving", 5}, {"Robotics", 5}};

  print_map("1) Initial map: ", m);

  m["Autonomous Driving"] = 10;
  m["Drone"] = 5;
  m["VR/AR"] = 8;
  print_map("2) Updated map: ", m);

  std::cout << "3) m[Javascript] = " << m["Javascript"] << "\n";
  print_map("4) Updated map: ", m);

  m.erase("Drone");
  print_map("5) After erase: ", m);
  std::cout << "6) m.size() = " << m.size() << "\n";

  m.clear();
  std::cout << std::boolalpha << "7) Map is empty: " << m.empty() << "\n";
}
