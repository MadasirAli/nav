#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <utility>
#include <array>

namespace nav {
  //class nav_engine
  //{
  //public:
  //  struct config {
  //    uint32_t fallHeight = -1;
  //  };

  //private:
  //  enum class neighbour_index {
  //    left,
  //    right,
  //    top,
  //    bottom
  //  };
  //  struct node {
  //    uint32_t x = 0;
  //    uint32_t y = 0;

  //    uint32_t g = 0;
  //    uint32_t h = 0;
  //    uint32_t cost = 0;

  //    uint32_t get_f() const {
  //      return (g + h + cost);
  //    }
  //  };
  //private:
  //  static constexpr const size_t _max_neighbours = 4;
  //  static constexpr const size_t _find_nodes_reserve = 256;
  //public:
  //  nav_engine(uint32_t width, uint32_t height);
  //  std::vector<std::pair<uint32_t, uint32_t>> find(uint32_t aX, uint32_t aY, uint32_t bX, uint32_t bY);

  //  void set_costs(std::unique_ptr<uint32_t[]>&& pCosts);
  //  void set_config(const config& config);
  //private:
  //  void reset_graph();

  //  void update_neighbours(uint32_t x, uint32_t y, uint32_t dX, uint32_t dY, node lastNode);
  //  std::pair<uint32_t, uint32_t> get_optimal_node() const;

  //  inline uint32_t calc_distance(uint32_t x, uint32_t y, uint32_t dX, uint32_t dY);

  //private:
  //  const uint32_t _width;
  //  const uint32_t _height;
  //  std::unique_ptr<node[]> _pNodes;
  //  std::unique_ptr<uint32_t[]> _pCosts;

  //  config _config = { 0 };
  //};
}

