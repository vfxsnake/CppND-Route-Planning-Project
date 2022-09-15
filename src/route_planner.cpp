#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbour: current_node->neighbors)
    {
        neighbour->parent = current_node;
        neighbour->h_value = CalculateHValue(neighbour);
        // not complety sure of this, it could be the current node g_value + the distance to neighbour
        neighbour->g_value += neighbour->distance(*current_node);
        neighbour->visited = true;
        open_list.push_back(neighbour);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort by lamba sort(vector_start, vector_end, [option](element_1, element_2){return result})
    sort(open_list.begin(), open_list.end(), 
         [](RouteModel::Node* a, RouteModel::Node* b)
         {return a->CalculateCost()>b->CalculateCost();});
    RouteModel::Node* lower_cost = open_list.back();
    open_list.pop_back();
    return lower_cost;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *node = current_node;
    // TODO: Implement your solution here.
    while (node)
    {
        path_found.insert(path_found.begin(), *node);
        if (node->parent)
            distance += node->distance(*(node->parent));
        node = node->parent;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    AddNeighbors(start_node);
    std::cout << "openlist size: " << open_list.size() << std::endl;
    while(open_list.size() > 0)
    {
        current_node = NextNode();
        std::cout << "openlist size after next node: " << open_list.size() << std::endl;
        std::cout << "current x, y" << current_node->x << current_node->y << std::endl; 
        if (current_node->x == end_node->x && current_node->y == end_node->y)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // AddNeighbors(current_node);
    }

}




// /////////////////////// delete me/////////////
// vector<vector<State>> Search(vector<vector<State>> grid, int init[2], int goal[2]) {
//   // Create the vector of open nodes.
//   vector<vector<int>> open {};
  
//   // Initialize the starting node.
//   int x = init[0];
//   int y = init[1];
//   int g = 0;
//   int h = Heuristic(x, y, goal[0],goal[1]);
//   AddToOpen(x, y, g, h, open, grid);

//   while (open.size() > 0) {
//     // Get the next node
//     CellSort(&open);
//     auto current = open.back();
//     open.pop_back();
//     x = current[0];
//     y = current[1];
//     grid[x][y] = State::kPath;

//     // Check if we're done.
//     if (x == goal[0] && y == goal[1]) {
//       // TODO: Set the init grid cell to kStart, and 
      
//       // set the goal grid cell to kFinish before returning the grid. 
//       grid[init[0]][init[1]] = State::kStart;
//       grid[goal[0]][goal[1]] = State::kFinish;
//       return grid;
//     }
    
//     // If we're not done, expand search to current node's neighbors.
//     ExpandNeighbors(current, goal, open, grid);
//   }
  
//   // We've run out of new nodes to explore and haven't found a path.
//   cout << "No path found!" << "\n";
//   return std::vector<vector<State>>{};
// }