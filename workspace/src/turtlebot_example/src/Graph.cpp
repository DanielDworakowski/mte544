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
//
///////////////////// A* algorithm////////////////////////////
std::stack<coord> Graph::aStar() {
    //
    //
    coord zeros = std::make_pair(0,0);
    vertex goalNode(zeros), startNode(zeros), temp(zeros);
    bool goalReached = false;
    std::stack<coord> bestPath;
    //
    startNode.loc = m_start;
    //
    //main loop that runs for 3 times
    for(auto & wayPoint : m_goals){
        goalNode.loc = wayPoint;
        std::priority_queue<vertex, std::vector<vertex>, LessThanByFullCost> openList;
        std::vector<vertex> closedList;
        //
        openList.push(startNode);
        //
        while(!openList.empty()){
            //
            //pop top of the openList
            check_:
            auto currentNode = openList.top();
            openList.pop();
            //
            //change to more effienceint
            for(auto & node : closedList){
                if (node.loc == currentNode.loc){
                    goto check_;
                }
            }
            //
            //things in the adjency list
            for(auto & aNode : currentNode.adj){
                //
                //calculate costs
                aNode.second->h = std::sqrt(std::pow((aNode.second->loc.first - goalNode.loc.first), 2.0)+std::pow((aNode.second->loc.second - goalNode.loc.second), 2.0));
                aNode.second->g = aNode.first + currentNode.g;
                aNode.second->f = aNode.second->g + aNode.second->h;
                //point parent to currentNode
                aNode.second->parent = &currentNode;
                //
                // push adjecent nodes into openList
                openList.push(*aNode.second);
            }
            //
            //push currentNode to closed list
            closedList.push_back(currentNode);
            //
            //check if it reached its goalNode
            if(currentNode.loc == goalNode.loc){
                goalNode = currentNode; // update to get cost and parent
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
        startNode.loc = wayPoint;
        startNode.parent =  temp.parent;
        startNode.g = 0.0;
    }

    return bestPath;
}
