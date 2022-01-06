// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>
#include <functional>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <memory>
// Types for IDs
using PlaceID = long int;
using AreaID = long int;
using Name = std::string;
using WayID = std::string;

// Return values for cases where required thing was not found
PlaceID const NO_PLACE = -1;
AreaID const NO_AREA = -1;
WayID const NO_WAY = "!!No way!!";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Enumeration for different place types
// !!Note since this is a C++11 "scoped enumeration", you'll have to refer to
// individual values as PlaceType::SHELTER etc.
enum class PlaceType { OTHER=0, FIREPIT, SHELTER, PARKING, PEAK, BAY, AREA, NO_TYPE };

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.x*c1.x + c1.y*c1.y < c2.x*c2.x + c2.y*c2.y)
        {return true;}
    else if (c1.x*c1.x + c1.y*c1.y > c2.x*c2.x + c2.y*c2.y)
        {return false;}
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Duration is unknown
Distance const NO_DISTANCE = NO_VALUE;



// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: Θ(1)
    // Short rationale for estimate: Constant time is required to return a vector's size
    int place_count();

    // Estimate of performance: O(n) where n is total number of places and areas
    // Short rationale for estimate: Clear subplaces and subareas of every places and areas
    void clear_all();

    // Estimate of performance: Θ(n)
    // Short rationale for estimate: Go through every elements of a vector requires
    // linear amount of time with vector's size
    std::vector<PlaceID> all_places();

    // Estimate of performance: O(logn)
    // Short rationale for estimate: Add new element to 2 multimaps and 1 unordered map
    bool add_place(PlaceID id, Name const& name, PlaceType type, Coord xy);

    // Estimate of performance: Θ(1)
    // Short rationale for estimate: Searching a key in an unordered map
    // requires constant time
    std::pair<Name, PlaceType> get_place_name_type(PlaceID id);

    // Estimate of performance: Θ(1)
    // Short rationale for estimate: earching a key in an unordered map
    // requires constant time
    Coord get_place_coord(PlaceID id);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: Θ(n)
    // Short rationale for estimate: Iterate through a multimap requires linear time
    std::vector<PlaceID> places_alphabetically();

    // Estimate of performance: O(nlogn)
    // Short rationale for estimate: Sorting a vector
    std::vector<PlaceID> places_coord_order();

    // Estimate of performance: O(logn) then O(n)
    // Short rationale for estimate: searching multimap using equal_range
    // costs logn. Insert to a vector costs n
    std::vector<PlaceID> find_places_name(Name const& name);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Search a map with key, copy values from an
    // unordered_set to a vector.
    std::vector<PlaceID> find_places_type(PlaceType type);

    // Estimate of performance: O(logn)
    // Short rationale for estimate: Searching for a key in unordered_map
    // costs constant time. Erase and insert name to multimap costs logn
    bool change_place_name(PlaceID id, Name const& newname);

    // Estimate of performance: O(1)
    // Short rationale for estimate: Searching with key in unordered_map
    // costs constant time
    bool change_place_coord(PlaceID id, Coord newcoord);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O(1)
    // Short rationale for estimate: Add new_element to unordered_map
    bool add_area(AreaID id, Name const& name, std::vector<Coord> coords);

    // Estimate of performance: O(1)
    // Short rationale for estimate: Search for key in unordered_map
    Name get_area_name(AreaID id);

    // Estimate of performance: O(1)
    // Short rationale for estimate: Search for key in unordered_map
    std::vector<Coord> get_area_coords(AreaID id);

    // Estimate of performance: Theta(n)
    // Short rationale for estimate: Iterate through all elements in map
    std::vector<AreaID> all_areas();

    // Estimate of performance: O(1)
    // Short rationale for estimate: Search with a key and assign
    // costs constant time
    bool add_subarea_to_area(AreaID id, AreaID parentid);

    // Estimate of performance: O(k) where k is number of parent areas
    // Short rationale for estimate: Iterate through all parent of an
    // area and add to a vector costs linear
    std::vector<AreaID> subarea_in_areas(AreaID id);

    // Non-compulsory operations

    // Estimate of performance: O(1)
    // Short rationale for estimate: Do nothing
    void creation_finished();

    // Estimate of performance: O(m^k) where m is number of direct
    // subareas, k is the last subarea of area id
    // Short rationale for estimate: Iterate through all direct subareas, add
    // them to a vector costs O(m), and doing it recursively costs (m^k)
    std::vector<AreaID> all_subareas_in_area(AreaID id);

    // Estimate of performance: O(n) then O(m) (m: places have the same type)
    // Short rationale for estimate: Without type parameter, nth_element
    // cost nlog(4); with type parameter; Iterate through all element
    // and add to a new vector cost O(n), then using nth_element costs O(m)
    std::vector<PlaceID> places_closest_to(Coord xy, PlaceType type);

    // Estimate of performance: O(logn)
    // Short rationale for estimate: Remove a value in unordered_set in unordered_map
    // and the last element in a vector costs constant time. Remove value in multimap
    // costs logn
    bool remove_place(PlaceID id);

    // Estimate of performance: Θ(min(n1,n2)) ( n is number of common parent areas)
    // Short rationale for estimate: Seach for common parent areas
    AreaID common_area_of_subareas(AreaID id1, AreaID id2);

private:
    using Id_TypeCoord_ptr = std::shared_ptr<std::pair<PlaceID, std::pair<PlaceType, Coord>>>;
    struct Place{
        PlaceID placeid;
        Name placename;
        PlaceType placetype;
        Coord placecoord;
        Id_TypeCoord_ptr placeid_typecoord_ptr;
        Place (const PlaceID &id_, const Name &name_, const PlaceType &type_, const Coord &coord_, const Id_TypeCoord_ptr &placeid_typecoord_ptr_){
            placeid = id_;
            placename = name_;
            placetype = type_;
            placecoord = coord_;
            placeid_typecoord_ptr = placeid_typecoord_ptr_;
        }
    };

    using Place_ptr = std::shared_ptr<Place>;
    std::multimap<Name, PlaceID> placename_to_placeid;
    std::unordered_map<PlaceType, std::unordered_set<PlaceID>> placetype_to_placeid;
    std::unordered_map<PlaceID, Place_ptr> placeid_to_data;
    std::vector<Id_TypeCoord_ptr> placeid_to_placetypecoord;

    bool is_sorted_placecoord_vector = false;

    struct Area{
        AreaID areaid;
        Name areaname;
        std::vector<Coord> areaboundary;
        std::shared_ptr<Area> parent_area = nullptr;
        std::unordered_map<AreaID, std::shared_ptr<Area>> subareas;
        Area(const AreaID &id_, const Name &name_, const std::vector<Coord> boundary_){
            areaid = id_;
            areaname = name_;
            areaboundary = boundary_;
        }
    };
    using Area_ptr = std::shared_ptr<Area>;
    std::unordered_map<AreaID, Area_ptr> areaid_to_data;
    std::vector<AreaID> recursive_subarea(AreaID id);
    double distance(Coord a, Coord b);
};

#endif // DATASTRUCTURES_HH
