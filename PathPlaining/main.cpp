#include <iostream>
#include "Graph.hpp"
#include "PathPlaining.hpp"

int main(int argc, char *argv[]){

    Graph gen;
    gen.generate_graph();
    gen.print_graph();

    PathSolver solver;
    solver.solve(0);

    return 0;
}
