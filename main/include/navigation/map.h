#pragma once
#include "action.h"
#include <string>
#include <deque>

struct map_object_t {
    std::string name;
    coords_t coords;
    PamiAction next_action;
    bool turnEnd;
    bool reverse;
    bool detect;

    bool operator==(const map_object_t& other) const {
        return name == other.name;
    }
};

class Map {
    private:
        std::deque<map_object_t> objects;
        // coords_t current_pos; 
    
    public:
        void add_object(map_object_t obj);
        void add_object(coords_t object_coords, std::string name, PamiAction next_action, bool turnEnd, bool reverse);
        void remove_object(map_object_t obj);
        void remove_object(std::string name);
        map_object_t find_object_by_name(std::string name);
        map_object_t get_next_object();
        /*
        map_object_t find_closest_object();
        */
};

