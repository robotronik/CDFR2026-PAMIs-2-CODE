#include "navigation/map.h"
#include "structs.h"
#include "math.h"
#include <algorithm>

/* Implements a map of waypoints with an associated action to do after reaching
 * said waypoint (next_action) using a double ended queue. Logic is FIFO.
 * Refer to action.
 */

void Map::add_object(map_object_t object) {
    objects.push_back(object);
}

void Map::add_object(coords_t object_coords, std::string name, PamiAction next_action, bool turnEnd, bool reverse, bool detect) {
    map_object_t new_object = { 
        .name = name,
        .coords = object_coords,
        .next_action = next_action,
        .turnEnd = turnEnd,
        .reverse = reverse, 
        .detect = detect
    };
    objects.push_back(new_object);    
}

void Map::remove_object(map_object_t object) {
    std::erase(objects, object);
}

void Map::remove_object(std::string name) {
    objects.erase(std::remove_if(objects.begin(), objects.end(), 
                                 [&name](const map_object_t& obj) { return obj.name == name; }),
                                objects.end());
} 

map_object_t Map::find_object_by_name(std::string name) {
    for(const auto& object : objects ) {
        if(object.name == name) {
            return object;
        }
    }    map_object_t result;
    return {};
}

map_object_t Map::get_next_object() {
    if(objects.empty()) {
        return {};
    }
    map_object_t object = objects.front();
    objects.pop_front();
    return object;
}

/*
map_object_t Map::find_closest_object() {
    map_object_t result;
    float lowest_distance = INFINITY;
    for(const auto& object: objects) {
        float delta_x = current_pos.x - object.position.x;
        float delta_y = current_pos.y - object.position.y;
        float distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        if(distance < lowest_distance) {
            lowest_distance = distance;
            result = object;
        }
    }
    return result;
}

*/
