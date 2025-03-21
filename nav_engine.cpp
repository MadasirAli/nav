#include "nav_engine.h"

#include <algorithm>
#include <cmath>
#include <assert.h>

using namespace nav;

//std::vector<std::pair<uint32_t, uint32_t>> 
//nav_engine::find(uint32_t aX, uint32_t aY, uint32_t bX, uint32_t bY)
//{
//  using namespace std;
//
//  reset_graph();
//
//  vector<pair<uint32_t, uint32_t>> result{};
//  result.reserve(_find_nodes_reserve);
//
//  uint32_t x = aX;
//  uint32_t y = aY;
//
//  uint32_t dX = bX;
//  uint32_t dY = bY;
//
//  node currentNode{};
//  node lastNode{};
//
//  while (true) {
//    update_neighbours(x, y, dX, dY, lastNode);
//    const auto optimal = get_optimal_node();
//
//    lastNode = currentNode;
//    currentNode = _pNodes[((size_t)optimal.second * _width) + optimal.first];
//
//    if (x == dX && y == dY) {
//      break;
//    }
//  }
//
//  return result;
//}
//
//void nav_engine::update_neighbours(uint32_t x, uint32_t y, uint32_t dX, uint32_t dY, node lastNode)
//{
//  using namespace std;
//
//  node& node = _pNodes.get()[((size_t)y * _width) + x];
//  node.g = calc_distance(x, y, x, y) + lastNode.g;
//  node.h = calc_distance(dX, dY, x, y);
//  node.cost = _pCosts.get()[((size_t)y * _width) + x ];
//
//  if (x != _width - 1) {
//    node& node = _pNodes.get()[((size_t)y * _width) + x + 1];
//    node.g = calc_distance(x, y, x + 1, y) + lastNode.g;
//    node.h = calc_distance(dX, dY, x, y);
//    node.cost = _pCosts.get()[((size_t)y * _width) + x + 1];
//  }
//  if (x != 0) {
//    node& node = _pNodes.get()[((size_t)y * _width) + x - 1];
//    node.g = calc_distance(x, y, x - 1, y) + lastNode.g;
//    node.h = calc_distance(dX, dY, x, y);
//    node.cost = _pCosts.get()[((size_t)y * _width) + x - 1];
//  }
//  if (y != 0) {
//    node& node = _pNodes.get()[((size_t)(y - 1) * _width) + x];
//    node.g = calc_distance(x, y, x, y -1) + lastNode.g;
//    node.h = calc_distance(dX, dY, x, y);
//    node.cost = _pCosts.get()[((size_t)(y - 1) * _width) + x];
//  }
//  if (y != _height - 1) {
//    node& node = _pNodes.get()[((size_t)(y + 1) * _width) + x];
//    node.g = calc_distance(x, y, x, y + 1) + lastNode.g;
//    node.h = calc_distance(dX, dY, x, y);
//    node.cost = _pCosts.get()[((size_t)(y + 1) * _width) + x];
//  }
//}
//
//std::pair<uint32_t, uint32_t> nav_engine::get_optimal_node() const
//{
//  using namespace std;
//
//  pair<uint32_t, uint32_t> result = make_pair(0, 0);
//
//  std::vector<node> nodes{};
//  nodes.reserve(_width * _height);
//
//  for (uint32_t i = 0; i < _width * _height; ++i) {
//    nodes[i] = _pNodes[i];
//  }
//
//  sort(nodes.begin(), nodes.end(), [](const node& a, const node& b) {
//    return a.get_f() < b.get_f();
//  });
//
//  std::vector<node> sames{};
//  for (uint32_t i = 1; i < _width * _height; ++i) {
//    if (nodes[i].get_f() == nodes[0].get_f()) {
//      sames.push_back(nodes[i]);
//    }
//  }
//
//  if (sames.size() > 0) {
//    sort(sames.begin(), sames.end(), [](const node& a, const node& b) {
//      return a.h < b.h;
//    });
//
//    result = make_pair(sames[0].x, sames[0].y);
//  }
//  else {
//    result = make_pair(nodes[0].x, nodes[0].y);
//  }
//
//  return result;
//}
//
//inline uint32_t nav_engine::calc_distance(uint32_t x, uint32_t y, uint32_t dX, uint32_t dY)
//{
//  return (std::sqrt(std::pow(dX - x, 2) + std::pow(dY - y, 2)));
//}
//
//nav_engine::nav_engine(uint32_t width, uint32_t height) :
//  _width(width),
//  _height(height),
//  _pNodes(std::make_unique<node[]>((size_t)_width * _height)),
//  _pCosts(std::make_unique<uint32_t[]>((size_t)_width* _height))
//{}
//
//void nav_engine::set_costs(std::unique_ptr<uint32_t[]>&& pCosts)
//{
//  _pCosts = std::move(pCosts);
//}
//
//void nav_engine::set_config(const config& config)
//{
//  _config = config;
//}
//
//void nav_engine::reset_graph()
//{
//  std::memset(_pNodes.get(), 0, sizeof(uint32_t) * _width * _height);
//
//  for (size_t i = 0; i < _height; ++i) {
//    for (size_t j = 0; j < _width; ++j) {
//      _pNodes[(i * _width) + j].x = j;
//      _pNodes[(i * _width) + j].y = i;
//    }
//  }
//}
