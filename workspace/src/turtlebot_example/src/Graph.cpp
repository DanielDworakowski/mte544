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
    vertex goalNode, startNode, temp;
    bool goalReached = false;
    std::stack<vertex> bestPath;
    //
    startNode.loc = m_start;
    //
    //main loop that runs for 3 times
    for(auto & wayPoint : m_goals){
        goalNode.loc = wayPoint;
        std::priority_queue<node, std::vector<node>, LessThanByFullCost> openList;
        std::vector<node> closedList;
        //
        openList.push(startNode);
        //
        while(!openList.empty()){
            //
            //pop top of the openList
            check_:
            auto currentNode = openList.pop();
            //
            //change to more effienceint
            for(auto & node : closedList){
                if (node.loc == currentNode.loc){
                    goto check_;
                }
            }
            //
            //things in the adjency list
            for(auto & aNode : currentNode.second->adj){
                //
                //calculate costs
                aNode.h = sqrt(pow((aNode.loc.fist - goalNode.loc.first), 2.0)+pow((aNode.loc.second - goalNode.loc.second), 2.0));
                aNode.g = aNode.first->adj + currentNode.g;///need to verify
                aNode.f = aNode.g + aNode.h;
                //point parent to currentNode
                aNode.parent = &currentNode;
                //
                // push adjecent nodes into openList
                openList.push(aNode);
            }
            //
            //push currentNode to closed list
            closedList.push_back(currentNode);
            //
            //check if it reached its goalNode
            if(currentNode.loc == goalNode.loc){
                goalNode = currentNode; // update to get cost and parent
                goalReached = true;
                break;
            }
        }
        //
        //check if there is no path
        if(!goalReached){
            std::cout << "No path has been found\n";
            return false;
        }
        //
        //
        temp = goalNode;
        //
        //decrale a list of flots
        while(temp.loc != startNode.loc){
            //add to the list
            bestPath.push_back(temp.loc);
            temp = temp->parent;//Aajksjdjsakjsdkljsdalkjas
        }
        startNode.loc = wayPoint;
        startNode.parent =  temp.parent;
        startNode.g = 0.0;
    }

    return bestPath;
}
