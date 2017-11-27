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
// //
// // A* algorithm
// bool aStar() {
//     //
//     // declare start and end
//     node startNode = (parent_, x_, y_, h_, g_, f_ );
//     node goalNode = (parent_, x_, y_, h_, g_, f_ );
//     //
//     // initialize 2 lists
//     std::priority_queue<node, std::vector<node>, LessThanByFullCost> openList;
//     std::vector<node> closedList;
//     //
//     openList.push(startNode);
//     //
//     while(!openList.empty()){
//       //
//       //pop top of the openList
//       if(openList.top().loc ==
//       openList.pop()
//
//       auto current = openList.pop();
//
//       //
//       //things in the adjency list //dont go to nodes in the closedList
//       for(auto & vtx : current.second->adj){
//         //
//         //calculate h
//         neighbour.h = sqrt(pow((neighbour.first - goalNode.first), 2.0) + pow((neighbour.second - goalNode.second), 2.0));
//         //
//         // calculate neighbor's g  and f
//         neighbour.g = cost ; // WRONG need to fix
//         neighbour.f = neighbour.g + neighbour.//attempt at finding the node with least fh;
//         //
//         //!!! if exisiting g in the openList greater than new g, update g and parent
//         if (temp.g < neighbourInTheOL.g){
//           //look in the openList and Update
//           neighbour.g = temp.g;
//           neighbour.parent = ffdasfdsa//pointerShit PARENT BECOME CURRENT
//         }
//         //
//         // push neighbour nodes into openList
//         if(){
//           openList.push(neighbour);
//         }
//       }
//       //
//       //push current to closed list
//       closedList.push_back(current);
//       //
//       //check if it reached its goalNode
//       if(current == goalNode){
//         goalNode = current; // update to get cost and parent
//         goalReached == true;
//         break;
//       }
//     }
//     //
//     //check if there is no path
//     if(!goalReached){
//       std::cout << "No path has been found\n";
//       return false
//     }
//     //
//     //cost of path is the cost of last element
//     node temp = goalNode;
//     //decrale a list of flots
//     while(temp != startNode){
//       //add to the list
//       temp = temp parent; //POINTER SHITE
//     }
//     return path;
//     //OR could just return the goalNode, or would the thing
// }
