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
    std::string anchorID;
    int distance;
};
/**
 * @brief Stores all of the different error flags obtained from the server.
*/
struct errorFlags
{
    std::string flag;
};

/**
 * @brief Stores the Tag`s X,Y,Z Coordinates calculated by the Anchors. 
*/
struct coordinates{
    float X;
    float Y;
    float Z;
};

struct Keyword
{
    std::string keyword;
    uint8_t update_intervalD;//Desired
    uint8_t update_intervalC;//Configured
    uint8_t update_intervalS;//Status

};


//To save the distance between each anchor an the Tag
struct rr_l
{
    uint8_t seqNumber;
    std::string tagSN;
    DistanceToAnchor anchors[10];
    std::string timestamp;
    errorFlags flags[10];
    errorFlags tagError;
};
// To save the Tags position calculated in meters 
struct coord
{
    uint8_t seqNumber;
    std::string tagSN;
    coordinates tagCoords;
    std::string Info;
    double timestamp;

};

// Like COORD but with some more details
struct coord_e
{
    coord coords;
    std::string IP;
    //This three parameters save the Estimates for the accuracy in meters;
    double Xconfidence;
    double Yconfidence;
    double Zconfidence;
};
/**
 * @brief Stores the values from the fields obtained when asking for the Anchor Coords
*/
struct ANCHOR_COORD
{
    std::string anchorID;
    char anchorRole;
    std::string anchorSN;
    coordinates anchorPosition;
    std::string lastConnectionEstablished;
    std::string lastConnectionLost;
    int connectionState;
};
/**
 *  @brief This struct stores more information about the Anchors Coords
*/
struct ANCHOR_COORD_E
{
    std::string anchorID;
    char anchorRole;
    std::string anchorSN;
    uint8_t cellSize;
    double hardwareVersion;
    std::string anchorSWVersion;
    std::string anchorModelString;
    coordinates anchorPosition;
    std::string lastConnectionEstablished;
    std::string lastConnectionLost;
    int connectionState;
};


struct TAG{
    std::string tagSN;
    std::string alias;
    std::string positionMode;
    double fixedHeight;
    double updateRate;
    std::string timestamp;
    coordinates lastPosition;
};

struct TAG_BATTERY
{
    std::string tagSN;
    std::string alias;
    uint16_t batteryVolt;
    std::string batteryStatus;
    std::string timestamp;
};

struct TAG_FILTERS
{
    std::string tagSN;
    std::string alias;
    char filterType;
    uint8_t distanceFilterLength;
    uint8_t firstCoordFilterLength;
    uint8_t secondCoordFilterLength;
    uint8_t distanceFilterLengthMS;
    uint8_t firstCoordFilterLengthMS;
    uint8_t secondCoordFilterLengthMS;
};

struct TAG_SAMPLE_INTERVALS
{
    std::string tagSN;
    std::string alias;
    Keyword High;
    Keyword Low;

};

