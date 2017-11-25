#include <iostream>
#include <vector>
#include <map>
#include <string>

typedef std::pair<uint32_t, uint32_t> coord;
#define PRINT_CORD(c) std::cout << " x: " << c.first << << ", y: " c.second << std::endl;

//https://stackoverflow.com/questions/5493474/graph-implementation-c
struct vertex {
    typedef std::pair<double, vertex*> ve;
    std::vector<ve> adj; //cost of edge, destination vertex
    coord loc;
    vertex(coord s) : loc(s) {}
};

class Graph {
public:
    typedef std::map<coord, vertex *> vmap;
    vmap m_vtx;
    void addvertex(const coord&);
    void addedge(const coord& from, const coord& to, double cost);
    void print();
};
