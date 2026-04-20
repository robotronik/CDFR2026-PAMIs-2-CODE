#include "navigation/map.h"
#include "structs.h"
#include "constants.h"

Map::Map() {
       
}

void Map::add_object(map_object_t object) {
    objects.push_back(object);
}

void Map::remove_object(map_object_t object) {
    std::erase(objects, object);
}

map_object_t Map::find_closest_object() {
    map_object_t result;
    for(const auto& object: objects) {
        
    }
    return result;
}

map_object_t Map::find_object_by_name(std::string name) {
    map_object_t result;
    for(const auto& object : objects ) {
        if(object.name == name) {
            result = object;
        }
    }
    return result;
}
