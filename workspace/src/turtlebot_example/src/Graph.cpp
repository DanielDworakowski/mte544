#include <Graph.hpp>
#include <cstring>

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
//
///////////////////// A* algorithm////////////////////////////
std::stack<coord> Graph::aStar() {
    //
    //
    coord zeros = std::make_pair(0,0);
    vertex goalNode(zeros), startNode(zeros), temp(zeros);
    bool goalReached = false;
    std::stack<coord> bestPath;
    double tmpH = 0, tmpG = 0, tmpF = 0;
    //
    // startNode.loc = m_start;
    startNode = *m_vtx[m_start];
    //
    //main loop that runs for 3 times
    for(auto & wayPoint : m_goals){
        goalNode = *m_vtx[wayPoint];
        std::priority_queue<vertex *, std::vector<vertex*>, LessThanByFullCost> openList;
        uint32_t flat = 0;
        uint8_t * visited = new uint8_t[m_sz*m_sz];
        memset(visited, 0, m_sz*m_sz);
        //
        openList.push(&startNode);
        //
        //
        while(!openList.empty()){
          //
          //pop top of the openList
          auto currentNode = openList.top();
          openList.pop();
          //
          //things in the adjency list
          for(auto & aNode : currentNode->adj){
            //
            // Check if the node is already in the list.
            flat = FLATTEN(aNode.second->loc);
            if (visited[flat]) {
              continue;
            }
            //
            //calculate costs
            tmpH = m_rez*(std::sqrt(std::pow((double(aNode.second->loc.first) - double(goalNode.loc.first)), 2)+std::pow((double(aNode.second->loc.second) - double(goalNode.loc.second)), 2)));
            tmpG = aNode.first + (currentNode->g);
            tmpF = tmpG + tmpH;
            //
            // check if we found a better way to get to the node in the list.
            if (tmpF > aNode.second->f) {
              continue;
            }
            aNode.second->h = tmpH;
            aNode.second->g = tmpG;
            aNode.second->f = tmpF;
            //point parent to currentNode
            aNode.second->parent = currentNode;
            //
            // push adjecent nodes into openList
            openList.push(aNode.second);
          }
          //
          //push currentNode to closed list
          flat = FLATTEN(currentNode->loc);
          visited[flat] = 1;
          //
          //check if it reached its goalNode
          if(currentNode->loc == goalNode.loc){
              goalNode = *currentNode; // update to get cost and parent
              goalReached = 1;
              break;
          }
        }

        //
        //check if there is no path
        if(!goalReached){
            throw std::logic_error( "No path has been found\n" );
        }
        //
        //
        temp = goalNode;
        //
        //decrale a list of flots
        while(temp.loc != startNode.loc){
          //add to the list
          bestPath.push(temp.loc);
          temp = *temp.parent;
        }
        bestPath.push(temp.loc);
        #pragma message("Pass adjacency")
        startNode = *m_vtx[wayPoint];
        startNode.parent =  temp.parent;
        startNode.g = 0.0;
        std::cout << "DONE A* loop\n";
        delete visited;
    }

    return bestPath;
}
