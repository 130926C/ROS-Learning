#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#define INFINITE 65535

#define MAX_GRAPH_WIDTH 50
#define MAX_GRAPH_LENGTH 50


#include <stdio.h>
#include <stdlib.h>
#include <iostream>

class Node{
public:
    Node():x(0), y(0) {};
    Node(int _x, int _y): x(_x), y(_y) {};

    bool operator==(const Node& n){
        if (this->x == n.x && this->y == n.y) return true;
        else return false;
    }

    bool operator!=(const Node& n){
        return !(*this == n);
    }

    Node &operator=(const Node& n){
        if (&n == this) return *this;
        this->x = n.x;
        this->y = n.y;
        return *this;
    }   

    friend std::ostream &operator <<(std::ostream& os, const Node& n);

public:
    int x, y;
};

std::ostream &operator<<(std::ostream& os, const Node & n){
    os << "Point [" << n.x << "," << n.y << "]" ;
    return os;
}


class Graph{
public:
    Graph():graph_width(MAX_GRAPH_WIDTH), graph_length(MAX_GRAPH_LENGTH) {};
    Graph(size_t _width, size_t _length){
        if (_width > MAX_GRAPH_WIDTH) printf("Warrning: width extend");
        if (_length > MAX_GRAPH_LENGTH) printf("Warrning: length extend");
        graph_width = _width > MAX_GRAPH_WIDTH ? MAX_GRAPH_WIDTH : _width;
        graph_length = _length > MAX_GRAPH_LENGTH ? MAX_GRAPH_LENGTH : _length; 
    }

    void PrintGraph(){
        for(int i = 0; i < graph_width; ++i){
            for(int j = 0; j < graph_length; ++j){
                if (graph[i][j] == INFINITE) printf("_\t");
                else printf("%d\t", graph[i][j]);
            }
            printf("\n");
        }
    }

private:
    size_t graph_width;
    size_t graph_length;
    
    int graph[MAX_GRAPH_WIDTH][MAX_GRAPH_LENGTH];
};


#endif 