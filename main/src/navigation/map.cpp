#include "navigation/map.h"
#include "structs.h"

Map::Map() {
       
}

void Map::add_object(map_object_t object) {
    objects.push_back(object);
}

void Map::remove_object(map_object_t object) {
    std::erase(objects, object);
}

map_object_t Map::find_closest_object() {
    return;
}

map_object_t Map::find_object_by_name() {
    return;
}
