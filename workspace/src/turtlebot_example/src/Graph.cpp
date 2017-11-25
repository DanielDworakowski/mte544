#include <Graph.hpp>

///////////////////////////////////////////////////////////////
void Graph::addvertex(
  const coord &name
)
{
    vmap::iterator itr = m_vtx.find(name);
    if (itr == m_vtx.end()) {
        vertex *v;
        v = new vertex(name);
        m_vtx[name] = v;
        return;
    }
}

///////////////////////////////////////////////////////////////
void Graph::addedge (
  const coord& from,
  const coord& to,
  double cost
)
{
    vertex *f = (m_vtx.find(from)->second);
    vertex *t = (m_vtx.find(to)->second);
    std::pair<double, vertex *> edge = std::make_pair(cost, t);
    f->adj.push_back(edge);
}

///////////////////////////////////////////////////////////////
void Graph::print (

)
{
  for (auto & kv : m_vtx) {
      std::cout << "Coord: (" << kv.first.first << ", " << kv.first.second << ")" << std::endl;
      for (auto & vtx : kv.second->adj) {
        std::cout << "\tConnect: (" << vtx.second->loc.first << ", " << vtx.second->loc.second << ") cost: " << vtx.first << std::endl;
      }
  }
}
