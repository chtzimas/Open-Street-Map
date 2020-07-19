#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) 
{
    //std::cout << "Enter RoutePlanner ctor\n";
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    //std::cout << "Initialize start_node and end_node\n";
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    //std::cout << "Exit RoutePlanner ctor\n";
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    //std::cout << "Enter CalculateHValue\n";
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    //std::cout << "Enter AddNeighbors\n";
    current_node->FindNeighbors();
    for (int i = 0; i < current_node->neighbors.size(); ++i)
    {
        float current_neighbor_distance = current_node->distance(*(current_node->neighbors)[i]);
        (current_node->neighbors)[i]->parent = current_node;
        (current_node->neighbors)[i]->g_value = current_node->g_value + current_neighbor_distance;
        (current_node->neighbors)[i]->h_value = this->CalculateHValue((current_node->neighbors)[i]);
        open_list.emplace_back((current_node->neighbors)[i]);
        (current_node->neighbors)[i]->visited = true;
    }
    //std::cout << "Exit AddNeighbors\n";   
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool RoutePlanner::Compare(const RouteModel::Node* node1, const RouteModel::Node* node2)
{
    //std::cout << "RoutePlanner::Compare\n";
    float f1 = node1->g_value + node1->h_value;
    float f2 = node2->g_value + node2->h_value;
    return f1 > f2;
}

void RoutePlanner::SortList(std::vector<RouteModel::Node*> *open_list)
{
    //std::cout << "RoutePlanner::SortList\n";
    sort(open_list->begin(), open_list->end(), Compare);
}

RouteModel::Node *RoutePlanner::NextNode() 
{
    //std::cout << "RoutePlanner::NextNode\n";
    this->SortList(&open_list);
    RouteModel::Node *closest_neighbor = open_list.back();
    open_list.pop_back();
    return closest_neighbor;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) 
{
    //std::cout << "Enter ConstructFinalPath\n";
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    path_found.emplace_back(*current_node);
    while(current_node != start_node)
    {   
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
        path_found.insert(path_found.begin(), *current_node);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    //std::cout << "Exit ConstructFinalPath\n";
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() 
{
    //std::cout << "Enter AStarSearch\n";
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    current_node = start_node;
    //std::cout << "AddNeighbors to start node\n";
    this->AddNeighbors(current_node);
    //std::cout << "Open nodes : " << open_list.size() << "\n";
    //std::cout << "Right before while loop\n";
    while(this->open_list.size() > 0)
    {
        current_node = this->NextNode();
        //std::cout << "Open nodes : " << open_list.size() << "\n";
        if (current_node == end_node)
        {
            m_Model.path = this->ConstructFinalPath(current_node);
            //std::cout << "Open nodes : " << open_list.size() << "\n";
            //std::cout << "Exit AStarSearch \n";
            //std::cout << "end_node found\n";
            return;    
        }
        
        this->AddNeighbors(current_node);
    }
    m_Model.path = this->ConstructFinalPath(current_node); 
    //std::cout << "Exit AStarSearch\n";
    //std::cout << "end_node not found\n";
    return;
}