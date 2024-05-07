/**
 * @file eliko_data.h
 * @brief This file contains all of the structures used by the driver to store and show the data obtained from the server.
*/
#include <cstdint>
#include <string>

/**
 * @brief Stores the calculated distance from the tag to the Anchor.
*/
struct DistanceToAnchor
{
  std::string anchor_id;
  int distance;
};
/**
 * @brief Stores all of the different error flags obtained from the server.
*/
struct ErrorFlags
{
  std::string flag;
};

/**
 * @brief Stores the Tag`s X,Y,Z Coordinates calculated by the Anchors. 
*/
struct Coordinates{
  float x;
  float y;
  float z;
};

struct Keyword
{
  std::string keyword;
  uint8_t update_interval_d;//Desired
  uint8_t update_interval_c;//Configured
  uint8_t update_interval_s;//Status

};


//To save the distance between each anchor an the Tag
struct Rr_l
{
  uint8_t seq_number;
  std::string tag_sn;
  DistanceToAnchor anchors[10];
  std::string timestamp;
  ErrorFlags flags[10];
  ErrorFlags tagError;
  uint8_t num_anchors;
};
// To save the Tags position calculated in meters 
struct Coord
{
  uint8_t seq_number;
  std::string tag_sn;
  Coordinates tag_coords;
  std::string info;
  double timestamp;
};

// Like COORD but with some more details
struct CoordE
{
  Coord coords;
  std::string ip;
  //This three parameters save the Estimates for the accuracy in meters;
  double x_confidence;
  double y_confidence;
  double z_confidence;
};
/**
 * @brief Stores the values from the fields obtained when asking for the Anchor Coords
*/
struct AnchorCoord
{
  std::string anchor_id;
  char anchor_role;
  std::string anchor_sn;
  Coordinates anchor_position;
  std::string last_connection_established;
  std::string last_connection_lost;
  int connection_state;
};
/**
 *  @brief This struct stores more information about the Anchors Coords
*/
struct AnchorCoordE
{
  std::string anchor_id;
  char anchor_role;
  std::string anchor_sn;
  uint8_t cell_size;
  double hardware_version;
  std::string anchor_sw_version;
  std::string anchor_model_string;
  Coordinates anchor_position;
  std::string last_connection_established;
  std::string last_connection_lost;
  int connection_state;
};


struct Tag
{
  std::string tag_sn;
  std::string alias;
  std::string position_mode;
  double fixed_height;
  double update_rate;
  std::string timestamp;
  Coordinates last_position;
};

struct TagBattery
{
  std::string tag_sn;
  std::string alias;
  uint16_t battery_volt;
  std::string battery_status;
  std::string timestamp;
};

struct TagFilters
{
  std::string tag_sn;
  std::string alias;
  char filter_type;
  uint8_t distance_filter_length;
  uint8_t first_coord_filter_length;
  uint8_t second_coord_filter_length;
  uint8_t distance_filter_length_ms;
  uint8_t first_coord_filter_length_ms;
  uint8_t second_coord_filter_length_ms;
};

struct TagSampleIntervals
{
  std::string tag_sn;
  std::string alias;
  Keyword high;
  Keyword low;
};

