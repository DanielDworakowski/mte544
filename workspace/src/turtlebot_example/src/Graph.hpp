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
    vertex *parent; //does it need to point to null?
    float h,g,f; // h heuristic cost, g cost to go to that node + prev cost, f full cost
};

struct LessThanByFullCost{
  bool operator()(const vertex& old_, const vertex& new_) const
  {
    return old_.f < new_.f;
  }
};

class Graph {
public:
    typedef std::map<coord, vertex *> vmap;
    vmap m_vtx;
    void addvertex(const coord&);
    void addedge(const coord& from, const coord& to, double cost);
    void print();
    bool aStar();
    //
    // locations of the start and goals.
    coord m_start;
    std::vector<coord> m_goals;
    //
    // Constructor
    Graph(
      coord start,
      std::vector<coord> goals
    )
    : m_start(start)
    , m_goals(goals) {}
};
