#pragma once
#include <iostream>
#include <vector>

inline void PrintVector(const std::vector<double>& vect) {
  for (auto x : vect) {
    std::cout << x << std::endl;
  }
}
