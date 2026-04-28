#pragma once
#include "structs.h"
#include <string>
#include <vector>

class Map {
    private:
        std::vector<map_object_t> objects;
        // coords_t current_pos; 
    
    public:
        void add_object(map_object_t obj);
        void add_object(coords_t object_coords, std::string name);
        void remove_object(map_object_t obj);
        void remove_object(std::string name);
        map_object_t find_object_by_name(std::string name);
        /*
        map_object_t find_closest_object();
        */
};

