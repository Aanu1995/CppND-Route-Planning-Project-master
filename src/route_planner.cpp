#include "route_planner.h"
#include <algorithm>
#include <vector>

using std:: vector;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find the closest nodes to the starting and ending coordinates
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


// calculate h value of a node

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return end_node -> distance(*node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	// populate current_node neighbours vector
    current_node -> FindNeighbors();
  	
  	for(auto neighbor : current_node -> neighbors){
      // set parent, h_value and g_value of each node
      neighbor -> parent = current_node;
      
      float current_node_distance = current_node->distance(*neighbor);
      neighbor -> g_value = current_node->g_value + current_node_distance;
      
      neighbor -> h_value = CalculateHValue(neighbor);
      
      // add neighbour to open list
      open_list.push_back(neighbor);
      neighbor -> visited = true;
    }
}


bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
  return (a->h_value + a->g_value) > (b->h_value + b->g_value);
}

// method to sort open list

void SortOpenList(vector<RouteModel::Node*> *list) {
  sort(list->begin(), list->end(), Compare);
}


// Complete the NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {
   SortOpenList(&open_list);
   RouteModel::Node *lowest_node = open_list.back();
   open_list.pop_back();
   return lowest_node;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  	while(current_node -> parent != nullptr){
      path_found.push_back(*current_node);
      RouteModel::Node parent = *(current_node->parent);
      distance += current_node->distance(parent);
      current_node = current_node->parent;
    }
	
  	path_found.push_back(*current_node);
  	// makes the start node first and end node last by reverse the path found
  	reverse(path_found.begin(), path_found.end());
  	
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}



// Perform Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

  	start_node -> visited = true;
  	open_list.push_back(start_node);
  
  	while(open_list.size() > 0){
       // find the next neighbouring node
      RouteModel::Node *next_node = NextNode();
      
      if(end_node -> distance(*next_node) == 0){
         m_Model.path = ConstructFinalPath(end_node);
		 return;
      }
      
      AddNeighbors(next_node);
    }
  
  	
}