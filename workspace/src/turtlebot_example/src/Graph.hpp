#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <lab2Math.hpp>
#include <stack>
#include <queue>
#include <cmath>
#include <stdexcept>

typedef std::pair<uint32_t, uint32_t> coord;
#define PRINT_CORD(c) std::cout << #c << " x: " << c.first << ", y: " << c.second << std::endl;
#define FLATTEN(c) ((c).first * m_sz + (c).second)

//https://stackoverflow.com/questions/5493474/graph-implementation-c
struct vertex {
    typedef std::pair<double, vertex*> ve;
    std::vector<ve> adj; //cost of edge, destination vertex
    coord loc;
    vertex(coord s) : loc(s) {}
    vertex *parent = NULL; //does it need to point to null?
    double h=0.0,g=0.0,f=0.0; // h heuristic cost, g cost to go to that node + prev cost, f full cost
};

struct LessThanByFullCost{
  bool operator()(const vertex * old_, const vertex * new_) const
  {
    //return old_->f < new_->f;
     return old_->f > new_->f;
  }
};

class Graph {
public:
    typedef std::map<coord, vertex *> vmap;
    vmap m_vtx;
    void addvertex(const coord&);
    void addedge(const coord& from, const coord& to, double cost);
    void print();
    std::stack<coord> aStar();
    //
    // locations of the start and goals.
    coord m_start;
    std::vector<coord> m_goals;
    uint32_t m_sz;
    double m_rez;
    //
    // Constructor
    void setStartAndGoals(
      coord start,
      std::vector<coord> goals,
      uint32_t sz,
      double rez
    )
    {
      m_start = start;
      m_goals = goals;
      m_sz = sz;
      m_rez = rez;
    }
};
