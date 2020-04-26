#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;


	start_node = &(m_Model.FindClosestNode(start_x , start_y)) ; 
    end_node   = &(m_Model.FindClosestNode(end_x ,end_y)) ; 
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return (*node).distance(*end_node) ; 
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors() ; 

  for (RouteModel::Node * n_node : current_node->neighbors ) //neighbors is a vector of pointers to node
  { 
     n_node->parent  = current_node ; 
     n_node->h_value = CalculateHValue(n_node);
     n_node->g_value = current_node->g_value + current_node->distance(*n_node) ; 
     n_node->visited = true ; 
     open_list.push_back(n_node) ; 
     
  }
  
}


/**
 * Compare the F values of two nodes.
 */
bool Compare(const RouteModel::Node *a, const RouteModel::Node * b) {
  float f1 = a->g_value + a->h_value; // f1 = g1 + h1
  float f2 = b->g_value + b->h_value; // f2 = g2 + h2
  return f1 > f2; 
}


/**
 * Sort the two-dimensional vector of ints in descending order.
 */
void CellSort(std::vector<RouteModel::Node*> *v) {
  sort(v->begin(), v->end(), Compare);
}

RouteModel::Node *RoutePlanner::NextNode() {
  CellSort(&open_list) ; 
  RouteModel::Node* low_sum_node = open_list.back() ; 
  open_list.pop_back();
  return low_sum_node ; 
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    
    RouteModel::Node * temp_node = current_node; 
    path_found.push_back(*temp_node); 
    while (temp_node != start_node)
    {
      path_found.push_back(*(temp_node->parent)); 
      distance +=temp_node->distance(*(temp_node->parent))   ;
      temp_node = temp_node->parent ;
    }
	// Reverse the vector 
    std::reverse(path_found.begin(), path_found.end()); 
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  	open_list.push_back (start_node) ;
	start_node->visited= true ;

  	while (open_list.size() > 0)
    {
      current_node = NextNode() ; 
      if (current_node == end_node) 
      {
       	m_Model.path = ConstructFinalPath(current_node) ; 
        return ;
      }
      AddNeighbors(current_node) ;

      
    }
	return ; 

}