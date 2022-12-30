#include <iostream>

#include "python3.10/Python.h"
#include "apf_py_wrapper.hpp"

void ImportTest() {
  ArtificialPotentialField apfModule = ArtificialPotentialField();
}

int main() {
  Py_Initialize();
  //ComputeVelocity();
  ImportTest();
  std::cout << "deinit" << std::endl;
  Py_Finalize();
  return 0;
}