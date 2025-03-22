#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <array>

#include "vector2.h"
#include "path.h"

namespace nav {
  class nav_engine
  {
  public:
    struct config {
      bool findDiagnals = false;
      //bool fallHeight = -1;
    };

  private:
    struct node {
      base::vector2_int pos = { 0,0 };
      base::vector2_int parent = { 0,0 };

      uint32_t cost = 0;

      bool is_open = false;
      bool is_closed = false;
    private:
      uint32_t _g = 0;
      uint32_t _h = 0;

    public:
      inline uint32_t get_g() const {
        return _g + cost;
      }
      inline uint32_t get_h() const {
        return _h;
      }
      inline uint32_t get_f() const {
        return (get_g() + get_h());
      }

      void set_g(uint32_t g) {
        _g = g;
      }
      void set_h(uint32_t h) {
        _h = h;
      }

      void reset_weights() {
        _g = 0;
        _h = 0;
      }
    };
    struct state {
      uint32_t openNodeCount = 0;
      base::vector2_int optimalOpenNode = { 0,0 };

      void reset() {
        openNodeCount = 0;
        optimalOpenNode = { 0,0 };
      }
    };

    static constexpr size_t _max_neighbours = 8;

  public:
    nav_engine(base::vector2_int size, const config& conf);

    void set_open(base::vector2_int pos, bool status);
    void get_neighbours(base::vector2_int pos, std::array<size_t, _max_neighbours>& out_result,
      bool getDiagnals);

    path find(base::vector2_int start, base::vector2_int end);

    void set_costs(const uint32_t* pCosts);
    void set_cost(base::vector2_int pos, uint32_t cost);

    void set_conf(const config& conf);

  private:
    void reset_graph();

  private:
    base::vector2_int _size;

    std::unique_ptr<node[]> _pNodes;

    config _config;
    state _state = { 0 };
  };
}