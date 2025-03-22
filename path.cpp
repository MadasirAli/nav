#include "path.h"

using namespace nav;

path::path(std::vector<base::vector2_int>&& nodes, bool valid) :
  nodes(nodes),
  valid(valid)
{}