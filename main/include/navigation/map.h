#pragma once
#include "rtos_wrapper.h"
#include "structs.h"
#include <vector>

class Map {
    private:
        std::vector<map_object_t> objects;
        coords_t current_pos; 
    
    public:
        void add_object(map_object_t obj);
        void remove_object(map_object_t obj);
        map_object_t find_closest_object();
        map_object_t find_object_by_name(std::string name);
};