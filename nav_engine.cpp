#include "nav_engine.h"

#include <cmath>
#include <assert.h>

using namespace nav;

path nav_engine::find(base::vector2_int start, base::vector2_int end)
{
  using namespace base;
  using namespace std;

  auto nodes = vector<vector2_int>();
  bool found = false;

  reset_graph();

  set_open(start, true);
  auto& startNode = _pNodes[start.to_index(_size.x)];
  startNode.set_h(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
  startNode.parent = { -1, -1 };

  while (_state.openNodeCount > 0) {

    const auto pos = _state.optimalOpenNode;
    auto& parent = _pNodes[pos.to_index(_size.x)];
    set_open(pos, false);

    if (pos == end) {
      found = true;
      break;
    }

    //// fall height
    //{
    //  if (parent.pos != vector2_int{ -1 ,-1 }) {
    //    const auto& grandParent = _pNodes[parent.parent.to_index(_size.x)];
    //    if (grandParent.pos != vector2_int{ -1, -1 }) {

    //    }
    //  }
    //}
    ////
    array<size_t, _max_neighbours > neighbours = {};
    get_neighbours(pos, neighbours, _config.findDiagnals);

    for (size_t i = 0; i < _max_neighbours; ++i) {
      if (neighbours[i] == -1) {
        continue;
      }

      auto& child = _pNodes[neighbours[i]];

      if (child.is_closed) {
        continue;
      }

      const bool opened = child.is_open;
      const auto cost = parent.get_g() + 
        (pow(child.pos.x - parent.pos.x, 2) + pow(child.pos.y - parent.pos.y, 2));

      if (!opened || child.get_g() < cost) {
        child.set_g(cost);
        child.parent = parent.pos;

        if (!opened) {
          set_open(child.pos, true);
          child.set_h(pow(child.pos.x - end.x, 2) + pow(child.pos.y - end.y, 2));
        }
      }
    }
  }

  nodes.push_back(end);
  vector2_int currentParent = _pNodes[end.to_index(_size.x)].parent;
  while (currentParent != start) {
    nodes.push_back(currentParent);
    currentParent = _pNodes[currentParent.to_index(_size.x)].parent;
  }
  nodes.push_back(start);

  return path{ move(nodes), found };
}

void nav_engine::set_open(base::vector2_int pos, bool status)
{
  auto& node = _pNodes[pos.to_index(_size.x)];

  if (status) {
    node.is_open = true;
    node.is_closed = false;

    if (_state.openNodeCount == 0) {
      _state.optimalOpenNode = pos;
    }
    else {
      const auto& lastOptNode = _pNodes[_state.optimalOpenNode.to_index(_size.x)];
      _state.optimalOpenNode = 
        node.get_f() < lastOptNode.get_f() ? pos :
          node.get_f() == lastOptNode.get_f() ?
          node.get_g() < lastOptNode.get_g() ? pos : _state.optimalOpenNode :
        _state.optimalOpenNode;
    }

    ++_state.openNodeCount;
  }
  else {
    node.is_open = false;
    node.is_closed = true;

    --_state.openNodeCount;
  }

  assert(_state.openNodeCount < 0 || _state.openNodeCount > _size.to_size());
}

void nav_engine::get_neighbours(base::vector2_int pos, std::array<size_t, _max_neighbours>& out_result,
  bool getDiagnals)
{
  const size_t null = -1;

  if (pos.x + 1 < _size.x) {
    out_result[0] = ((size_t)(pos.y) * _size.x) + pos.x + 1;
  }
  else {
    out_result[0] = null;
  }

  if (pos.x - 1 > -1) {
    out_result[1] =((size_t)(pos.y) * _size.x) + pos.x - 1;
  }
  else {
    out_result[1] = null;
  }

  if (pos.y + 1 < _size.y) {
    out_result[2] = ((size_t)(pos.y + 1) * _size.x) + pos.x;
  }
  else {
    out_result[2] = null;
  }

  if (pos.y - 1 > -1) {
    out_result[3] = ((size_t)(pos.y - 1) * _size.x) + pos.x;
  }
  else {
    out_result[3] = null;
  }

  if (getDiagnals) {
    if (pos.x + 1 < _size.x && pos.y + 1 < _size.y) {
      out_result[4] = ((size_t)(pos.y + 1) * _size.x) + pos.x + 1;
    }
    else {
      out_result[4] = null;
    }

    if (pos.x - 1 > -1 && pos.y + 1 < _size.y) {
      out_result[5] = ((size_t)(pos.y + 1) * _size.x) + pos.x - 1;
    }
    else {
      out_result[5] = null;
    }

    if (pos.x + 1 < _size.x && pos.y - 1 > -1) {
      out_result[6] = ((size_t)(pos.y - 1) * _size.x) + pos.x + 1;
    }
    else {
      out_result[6] = null;
    }

    if (pos.x - 1 > -1 && pos.y - 1 > -1) {
      out_result[7] = ((size_t)(pos.y - 1) * _size.x) + pos.x - 1;
    }
    else {
      out_result[7] = null;
    }
  }
  else {
    out_result[4] = null;
    out_result[5] = null;
    out_result[6] = null;
    out_result[7] = null;
  }
}

nav::nav_engine::nav_engine(base::vector2_int size, const config& conf) :
  _size(size),
  _pNodes(std::make_unique<node[]>(_size.to_size())),
  _config(conf)
{
  for (size_t i = 0; i < _size.to_size(); ++i) {
    _pNodes[i].pos = { (int)(i % size.x), (int)(i / size.y) };
  }
}

void nav::nav_engine::reset_graph()
{
  for (size_t i = 0; i < _size.to_size(); ++i) {
    _pNodes[i].reset_weights();
  }

  _state.reset();
}

void nav_engine::set_costs(const uint32_t* pCosts)
{
  for (size_t i = 0; i < _size.to_size(); ++i) {
    _pNodes[i].cost = pCosts[i];
  }
}

void nav_engine::set_cost(base::vector2_int pos, uint32_t cost)
{
  _pNodes[pos.to_index(_size.x)].cost = cost;
}

void nav_engine::set_conf(const config& conf)
{
  _config = conf;
}
