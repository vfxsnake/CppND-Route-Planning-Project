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
        open_list.push_back(neighbour);
        neighbour->visited = true;
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

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    AddNeighbors(start_node);
    while(open_list.size() > 0)
    {
        current_node = NextNode();
        if (current_node->x == end_node->x && current_node->y == end_node->y)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }

}