// Datastructures.cc

#include "datastructures.hh"

#include <random>
#include <algorithm>
#include <cmath>
#include <unordered_set>

using std::vector;
using std::pair;
using std::make_pair;
using std::unordered_set;

using std::shared_ptr;
using std::make_shared;

using std::sort;
using std::nth_element;
using std::multimap;

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end){
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures(){}

Datastructures::~Datastructures(){
    placeid_to_data.clear();
    placeid_to_placetypecoord.clear();
    placename_to_placeid.clear();
    placetype_to_placeid.clear();
    for (auto &area : areaid_to_data){
        area.second->subareas.clear();
        area.second->parent_area = nullptr;
    }
    areaid_to_data.clear();
}

int Datastructures::place_count(){
    return placeid_to_placetypecoord.size();
}

void Datastructures::clear_all(){
    placeid_to_placetypecoord.clear();
    placeid_to_data.clear();
    placename_to_placeid.clear();
    placetype_to_placeid.clear();
    is_sorted_placecoord_vector = false;
    areaid_to_data.clear();
}

vector<PlaceID> Datastructures::all_places(){
    vector<PlaceID> placeid_vector;
    placeid_vector.reserve(placeid_to_data.size());
    for (auto &data: placeid_to_placetypecoord)
        placeid_vector.push_back(data->first);
    return placeid_vector;
}

bool Datastructures::add_place(PlaceID id, const Name& name, PlaceType type, Coord xy){
    auto it = placeid_to_data.find(id);
    if (it != placeid_to_data.end())
        return false;
    pair<PlaceType, Coord>typecoord(type, xy);
    placeid_to_placetypecoord.push_back(make_shared<pair<PlaceID, pair <PlaceType, Coord>>>(id, typecoord));
    shared_ptr<Place> new_place = make_shared<Place>(id, name, type, xy, placeid_to_placetypecoord[placeid_to_placetypecoord.size()-1]);
    placeid_to_data.insert({id, new_place});
    placename_to_placeid.emplace(name, id);
    placetype_to_placeid[type].insert(id);

    is_sorted_placecoord_vector = false;
    return true;
}

pair<Name, PlaceType> Datastructures::get_place_name_type(PlaceID id){
    auto it = placeid_to_data.find(id);
    if (it == placeid_to_data.end()){
        pair<Name, PlaceType> noname(NO_NAME, PlaceType::NO_TYPE);
        return noname;
    }
    pair<Name, PlaceType> get_name_type(it->second->placename, it->second->placetype);
    return get_name_type;
}

Coord Datastructures::get_place_coord(PlaceID id){
    auto it = placeid_to_data.find(id);
    if (it == placeid_to_data.end()){
        return NO_COORD;
    }
    Coord get_coord = it->second->placecoord;
    return get_coord;
}

vector<PlaceID> Datastructures::places_alphabetically(){
    vector<PlaceID> placeid_vector;
    placeid_vector.reserve(placeid_to_placetypecoord.size());

    for (auto &name_id : placename_to_placeid)
        placeid_vector.push_back(name_id.second);
    return placeid_vector;
}

vector<PlaceID> Datastructures::places_coord_order(){

    if (is_sorted_placecoord_vector == false){
        sort(placeid_to_placetypecoord.begin(), placeid_to_placetypecoord.end(),
             [](const Id_TypeCoord_ptr &ptr_a, const Id_TypeCoord_ptr &ptr_b)->bool
                {return ptr_a->second.second < ptr_b->second.second;});
        is_sorted_placecoord_vector = true;
    }
    vector<PlaceID> placecoord_vector;
    placecoord_vector.reserve(placeid_to_placetypecoord.size());
    for (auto &id_coord : placeid_to_placetypecoord)
        placecoord_vector.push_back(id_coord->first);
    return placecoord_vector;
}

vector<PlaceID> Datastructures::find_places_name(Name const& name){
    vector<PlaceID> placeid_vector;
    auto it_pair = placename_to_placeid.equal_range(name);
    if (it_pair.first == placename_to_placeid.end())
        return {};
    for (auto it = it_pair.first; it != it_pair.second; it++)
        placeid_vector.push_back(it->second);
    return placeid_vector;

}

vector<PlaceID> Datastructures::find_places_type(PlaceType type){

    if (placetype_to_placeid.find(type) == placetype_to_placeid.end())
        return {};
    vector<PlaceID> same_type;
    same_type.reserve(placetype_to_placeid[type].size());
    for (auto &id: placetype_to_placeid[type])
        same_type.push_back(id);
    return same_type;
}

bool Datastructures::change_place_name(PlaceID id, const Name& newname){
    auto it = placeid_to_data.find(id);
    if (it == placeid_to_data.end())
        return false;

    auto pair = placename_to_placeid.equal_range(it->second->placename);
    for (auto iter = pair.first; iter != pair.second; iter++) {
        if (iter->second == id) {
            placename_to_placeid.insert(iter, {newname, id});
            placename_to_placeid.erase(iter);
            it->second->placename = newname;
            return true;
        }
    }
    it->second->placename = newname;
    return true;
}

bool Datastructures::change_place_coord(PlaceID id, Coord newcoord){
    auto it = placeid_to_data.find(id);
    if (it == placeid_to_data.end())
        return false;

    it ->second->placecoord = newcoord;
    it->second->placeid_typecoord_ptr->second.second = newcoord;
    is_sorted_placecoord_vector = false;
    return true;
}

bool Datastructures::add_area(AreaID id, const Name &name, vector<Coord> coords){
    auto it = areaid_to_data.find(id);
    if (it == areaid_to_data.end()){
        shared_ptr<Area> new_area = make_shared<Area>(id, name, coords);
        areaid_to_data.insert({id, new_area});
        return true;
    }
    return false;
}

Name Datastructures::get_area_name(AreaID id){
    auto it = areaid_to_data.find(id);
    if (it != areaid_to_data.end())
        return it->second->areaname;

    return NO_NAME;
}

vector<Coord> Datastructures::get_area_coords(AreaID id){
    auto it = areaid_to_data.find(id);
    if (it != areaid_to_data.end())
        return it->second->areaboundary;

    return {NO_COORD};
}

vector<AreaID> Datastructures::all_areas(){
    if (areaid_to_data.size() == 0)
        return {};
    vector<AreaID> areas;
    areas.reserve(areaid_to_data.size());
    for (auto &id_data : areaid_to_data)
        areas.push_back(id_data.first);
    return areas;
}

bool Datastructures::add_subarea_to_area(AreaID id, AreaID parentid){
    auto it = areaid_to_data.find(id);
    if (it != areaid_to_data.end()){
        auto parent_it = areaid_to_data.find(parentid);
        if (parent_it != areaid_to_data.end()){
            it->second->parent_area = parent_it->second;
            parent_it->second->subareas.insert({id, it->second});
            return true;
        }
    }
    return false;
}

vector<AreaID> Datastructures::subarea_in_areas(AreaID id){

    auto it = areaid_to_data.find(id);
    if (it != areaid_to_data.end()){
        vector<AreaID> areas;
        if (it->second->parent_area == nullptr)
            return {};
        else{
            Area_ptr parent_ptr = areaid_to_data.at(it->second->parent_area->areaid);
            while (parent_ptr != nullptr){
                areas.push_back(parent_ptr->areaid);
                parent_ptr = parent_ptr->parent_area;
            }
            return areas;
        }
    }
    return {NO_AREA};
}

void Datastructures::creation_finished(){}

vector<AreaID> Datastructures::recursive_subarea(AreaID id){
    vector<AreaID> subareas;
    auto it = areaid_to_data.find(id);
    if (it->second->subareas.size() == 0)
        return {};
    for (auto &s : it->second->subareas){
        subareas.push_back(s.first);
        for (auto x: recursive_subarea(s.first))
        subareas.push_back(x);
    }
    return subareas;
}

vector<AreaID> Datastructures::all_subareas_in_area(AreaID id){
    auto it = areaid_to_data.find(id);
    if (it != areaid_to_data.end()){
        return recursive_subarea(id);
    }
    return {NO_AREA};
}

double Datastructures::distance(Coord a, Coord b){
    return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
}

vector<PlaceID> Datastructures::places_closest_to(Coord xy, PlaceType type){
    vector<PlaceID> v;
    if (type == PlaceType::NO_TYPE){
        if (placeid_to_placetypecoord.size() <= 3){
            sort(placeid_to_placetypecoord.begin(), placeid_to_placetypecoord.end(),
                 [&](const Id_TypeCoord_ptr& ptr_a, const Id_TypeCoord_ptr& ptr_b)->bool{
                return distance(ptr_a->second.second, xy) < distance(ptr_b->second.second, xy);
            });

            for (auto &place: placeid_to_placetypecoord)
                v.push_back(place->first);
            return v;
        }
        nth_element(placeid_to_placetypecoord.begin(), placeid_to_placetypecoord.begin()+3, placeid_to_placetypecoord.end(),
                    [&](const Id_TypeCoord_ptr& ptr_a, const Id_TypeCoord_ptr& ptr_b)->bool{
                        return distance(ptr_a->second.second,xy)< distance(ptr_b->second.second, xy);
                    });

        sort(placeid_to_placetypecoord.begin(), placeid_to_placetypecoord.begin()+3,
             [&](const Id_TypeCoord_ptr& ptr_a, const Id_TypeCoord_ptr& ptr_b)->bool{
                    return distance(ptr_a->second.second, xy) < distance(ptr_b->second.second, xy);
                });
        for (int i = 0; i < 3; i++)
            v.push_back(placeid_to_placetypecoord[i]->first);
        return v;
    }
    else{
        // This vector contains all ptr that have required type.
        vector<Id_TypeCoord_ptr> same_type;
        for (auto &ptr: placeid_to_placetypecoord){
            if (ptr->second.first == type )
                same_type.push_back(ptr);
        }

        if (same_type.size() <= 3){
            for (unsigned long int j = 0 ;j < same_type.size(); j++)
                v.push_back(same_type[j]->first);
            return v;
        }
        nth_element(same_type.begin(), same_type.begin()+3, same_type.end(),
                    [&](const Id_TypeCoord_ptr& ptr_a, const Id_TypeCoord_ptr& ptr_b)->bool{
                            return distance(ptr_a->second.second, xy) < distance(ptr_b->second.second, xy);
                        });

        sort(same_type.begin(), same_type.begin()+3,
                    [&](const Id_TypeCoord_ptr& ptr_a, const Id_TypeCoord_ptr& ptr_b)->bool{
                            return distance(ptr_a->second.second, xy) < distance(ptr_b->second.second, xy);
                        });
        for (int i = 0; i < 3; i++)
            v.push_back(same_type[i]->first);
        return v;
    }

}

bool Datastructures::remove_place(PlaceID id){
    auto it = placeid_to_data.find(id);
    if (it == placeid_to_data.end())
        return false;
    if (placeid_to_placetypecoord.size() == 1){
        placeid_to_data.clear();
        placeid_to_placetypecoord.clear();
        placename_to_placeid.clear();
        placetype_to_placeid.clear();
        is_sorted_placecoord_vector = false;
        return true;
    }
    auto nameid_ptr = placename_to_placeid.equal_range(it->second->placename);
    auto name_ptr = nameid_ptr.first;
    for (auto ptr = nameid_ptr.first; ptr != nameid_ptr.second; ptr++){
        if (ptr->second == id){
            name_ptr = ptr;
            break;
        }
    }
    placename_to_placeid.erase(name_ptr);

    PlaceType it2 = placeid_to_data[id]->placetype;
    placetype_to_placeid[it2].erase(id);

    // The pointer which used to point the place pointer with largest Coord now points to the to be deleted place pointer
    placeid_to_data[placeid_to_placetypecoord[placeid_to_placetypecoord.size()-1]->first]->placeid_typecoord_ptr = it->second->placeid_typecoord_ptr;
    placeid_to_data.erase(it);

    // Swap the place pointer to be deleted to the last element for fast deletion: O(1)
    it->second->placeid_typecoord_ptr->second = placeid_to_placetypecoord[placeid_to_placetypecoord.size()-1]->second;
    it->second->placeid_typecoord_ptr->first = placeid_to_placetypecoord[placeid_to_placetypecoord.size()-1]->first;
    placeid_to_placetypecoord.pop_back();
    is_sorted_placecoord_vector = false;
    return true;
}

AreaID Datastructures::common_area_of_subareas(AreaID id1, AreaID id2)
{
    auto it1 = areaid_to_data.find(id1);
    auto it2 = areaid_to_data.find(id2);
    if (it1 == areaid_to_data.end() or it2 == areaid_to_data.end()
        or it1->second->parent_area == nullptr or it2->second->parent_area == nullptr)
        return NO_AREA;
    shared_ptr<Area> parent_ptr1 = areaid_to_data[it1->second->parent_area->areaid];
    shared_ptr<Area> parent_ptr2 = areaid_to_data[it2->second->parent_area->areaid];
    unordered_set<AreaID> parentareas1, parentareas2;
    while (parent_ptr1 != nullptr or parent_ptr2 != nullptr){
        if (parent_ptr1 != nullptr)
            parentareas1.insert(parent_ptr1->areaid);
        if (parent_ptr2 != nullptr)
            parentareas2.insert(parent_ptr2->areaid);

        // Find if an area's current parent is also other's
        if (parent_ptr1 != nullptr and parentareas2.find(parent_ptr1->areaid) != parentareas2.end())
            return parent_ptr1->areaid;
        if (parent_ptr2 != nullptr and parentareas1.find(parent_ptr2->areaid) != parentareas1.end())
            return parent_ptr2->areaid;

        // Increment
        if (parent_ptr1 != nullptr)
            parent_ptr1 = parent_ptr1->parent_area;
        if (parent_ptr2 != nullptr)
            parent_ptr2 = parent_ptr2->parent_area;
    }
    return NO_AREA;
}
