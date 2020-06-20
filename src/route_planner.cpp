#include "route_planner.h"
#include <algorithm>
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    RoutePlanner::start_node = &(m_Model.FindClosestNode(start_x, start_y));
    RoutePlanner::end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return end_node->distance(*node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors)
    {
        if (!neighbor->visited)
        {
            // set the neighbor's parent node
            neighbor->parent = current_node;

            // set the neighbor's g value
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);

            // set the neighbor's h vallue
            neighbor->h_value = CalculateHValue(neighbor);

            // visited neighbor
            neighbor->visited = true;

            // push neighbor node to open list
            open_list.push_back(neighbor);
        }
    }
}

bool Compare(RouteModel::Node *a, RouteModel::Node *b)
{
    auto f1 = a->g_value + a->h_value;
    auto f2 = b->g_value + b->h_value;

    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode()
{
    sort(open_list.begin(), open_list.end(), Compare);

    auto closest = open_list.back();

    open_list.pop_back();

    return closest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent)
    {

        path_found.push_back(*current_node);

        // get the parent node
        auto parent = current_node->parent;

        // add the distance between the current node and its parent
        distance = distance + current_node->distance(*parent);

        // update the current node to point to the parent
        current_node = parent;
    }

    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;
}

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    start_node->visited = true;

    while (open_list.size() > 0)
    {
        // get the next node
        current_node = NextNode();

        // check if we've reached the end
        if (current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // add neighbors to open list if search is not done
        AddNeighbors(current_node);
    }

    // If the search is not successful
    std::cout << "No path was found!" << std::endl;

    return;
}
