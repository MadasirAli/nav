#pragma once
#include <vector>

#include "vector2.h"

namespace nav {
  class path {
  public:
    const std::vector<base::vector2_int> nodes;
    const bool valid;

  public:
    path(std::vector<base::vector2_int>&& nodes, bool valid);
  };
}