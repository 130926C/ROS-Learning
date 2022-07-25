#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#define INFINITY 65535
#define GRAPH_WIDTH 20
#define GRAPH_LENGTH 20

#define PATH_EDGE 30
#define MAX_COST 30

#include <vector>
#include <iostream>
#include <stdio.h>

static std::vector<std::vector<int>> graph(GRAPH_WIDTH, std::vector<int>(GRAPH_LENGTH, INFINITY));


class Graph{
public:
    Graph():path_edge(PATH_EDGE), max_cost(MAX_COST) {};
    Graph(size_t path_count, size_t max_cost):path_edge(path_count), max_cost(max_cost) {
        if (path_edge > (GRAPH_WIDTH * GRAPH_LENGTH)) {
            printf("path edge is too more, init failed\n");
            return ;
        }
    };
    ~Graph() = default;

    void generate_graph(){
        int count = 0;
        while(count < this->path_edge){
            size_t i_index = rand() % GRAPH_WIDTH;
            size_t j_index = rand() % GRAPH_LENGTH;
            if (graph[i_index][j_index] == INFINITY){
                graph[i_index][j_index] = rand() % MAX_COST;
                ++count;
            }else continue;
        }
    }

    void print_graph(){
        printf("Map:\n");
        for(size_t i = 0; i < GRAPH_WIDTH; ++i){
            for(size_t j = 0; j< GRAPH_LENGTH; ++j){
                printf("%d\t", graph[i][j]);
            }
            printf("\n");
        }
    }

private:


private:
    size_t path_edge;
    size_t max_cost;
};

#endif 