#include <cstdint>

struct DistanceToAnchor
{
    std::string anchorID;
    int distance;
};

struct errorFlags
{
    std::string flag;
};

struct coordinates{
    double X;
    double Y;
    double Z;
};

struct Keyword
{
    std::string keyword;
    uint8_t update_intervalD;//Desired
    uint8_t update_intervalC;//Configured
    uint8_t update_intervalS;//Status

};

//To save the distance between each anchor an the Tag
struct RR_L
{
    uint8_t seqNumber;
    std::string tagID;
    DistanceToAnchor anchors[10];
    std::string timestamp;
    errorFlags flags[10];
    errorFlags tagError;
};
// To save the Tags position calculated in meters 
struct COORD
{
    uint8_t seqNumber;
    std::string tagSN;
    coordinates tagCoords;
    std::string Info;
    double timestamp;

};

// Like COORD but with some more details
struct COORD_E
{
    COORD coords;
    std::string IP;
    //This three parameters save the Estimates for the accuracy in meters;
    double Xconfidence;
    double Yconfidence;
    double Zconfidence;
};
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

