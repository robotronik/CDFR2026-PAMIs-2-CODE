#pragma once 
#include "structs.h"
#include <vector>

class Map {
    private:
        std::vector<map_object_t> objects;
    
    public:
        Map();
        void add_object(map_object_t obj);
        void remove_object(map_object_t obj);
        map_object_t find_closest_object();
        map_object_t find_object_by_name();

};
