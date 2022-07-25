#ifndef __PATH_PLAINING_HPP__
#define __PATH_PLAINING_HPP__

#include "Graph.hpp"
#include <vector>

extern std::vector<std::vector<int>> graph;

class PathSolver{
public:
    PathSolver() {};
    ~PathSolver() = default;

    // 对外开放的接口
    bool solve(int type);

private:
    bool __Dijstra__();
    bool __A_star__();
    bool __PRM__();
    bool __RRT__();
    bool __RRT_star__();
    bool __Inform_RRT_star__();
};

#endif