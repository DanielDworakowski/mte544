#include <iostream>
#include <cmath>
#include <list>
#include <vector>
#include <algorithm>
#include <Graph.hpp>
//jfoidajoifjdaisjfidsjaoifjidajsifjsdiajfdasfacdacderewtfadfdadcxczcxcsdafda
//DO I NEED A STD LIB?
//
// node struct
struct node {
  node *parent;
  int x, y;
  float h,g,f; //h heuristic, g cost of edge + predecesors, f full cost
}
//
//operator
