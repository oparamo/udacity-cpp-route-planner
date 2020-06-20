#include "render.h"
#include "route_model.h"
#include "route_planner.h"
#include <fstream>
#include <io2d.h>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
using std::byte;
using std::cin;
using std::cout;
using std::endl;
using std::ifstream;
using std::ios;
using std::move;
using std::nullopt;
using std::optional;
using std::string;
using std::string_view;
using std::vector;

using namespace std::experimental;

static optional<vector<byte>> ReadFile(const string &path)
{
    ifstream is{path, ios::binary | ios::ate};
    if (!is)
        return nullopt;

    auto size = is.tellg();
    vector<byte> contents(size);

    is.seekg(0);
    is.read((char *)contents.data(), size);

    if (contents.empty())
        return nullopt;

    return move(contents);
}

int main(int argc, const char **argv)
{
    string osm_data_file = "";
    if (argc > 1)
    {
        for (int i = 1; i < argc; ++i)
            if (string_view{argv[i]} == "-f" && ++i < argc)
                osm_data_file = argv[i];
    }
    else
    {
        cout << "To specify a map file use the following format: " << endl;
        cout << "Usage: [executable] [-f filename.osm]" << endl;
        osm_data_file = "../map.osm";
    }

    vector<byte> osm_data;

    if (osm_data.empty() && !osm_data_file.empty())
    {
        cout << "Reading OpenStreetMap data from the following file: " << osm_data_file << endl;
        auto data = ReadFile(osm_data_file);
        if (!data)
            cout << "Failed to read." << endl;
        else
            osm_data = move(*data);
    }

    float start_x;
    float start_y;
    float end_x;
    float end_y;

    cout << "Please enter the starting X position:" << endl;
    cin >> start_x;

    cout << "Please enter the starting Y position:" << endl;
    cin >> start_y;

    cout << "Please enter the ending X position:" << endl;
    cin >> end_x;

    cout << "Please enter the ending Y position:" << endl;
    cin >> end_y;

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface &surface) {
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface &surface) {
        render.Display(surface);
    });
    display.begin_show();
}
