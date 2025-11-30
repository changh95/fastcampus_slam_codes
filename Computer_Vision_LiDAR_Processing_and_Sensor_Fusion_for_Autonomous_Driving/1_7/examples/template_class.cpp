#include <iostream>

template <class T>
class Number {
private:
  T num;

public:
  Number(T n) : num(n) {}

  T getNum() {
    return num;
  }
};

int main()
{
  Number<int> numberInt(7);
  Number<double> numberDouble(7.7);

  std::cout << "int Number = " << numberInt.getNum() << std::endl;
  std::cout << "double Number = " << numberDouble.getNum() << std::endl;
}