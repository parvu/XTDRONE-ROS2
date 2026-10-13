#ifndef CONSTANTS
#define CONSTANTS

#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <string>

namespace HybridAStar {
namespace Constants {
extern bool coutDEBUG;                          
extern bool manual;                             
extern bool visualization;                      
extern bool visualization2D;                    
extern bool reverse;                            
extern bool dubinsShot;                         
extern bool dubins;                             
extern bool dubinsLookup;                       
extern bool twoD;                               


extern int iterations;                          
extern double bloating;                         
extern double width;                            
extern double length;                           
extern float r;                                 
extern float vehicle_wheelbase;                 
extern float vehicle_max_rudder;                
static const int headings = 72;                           
extern float deltaHeadingDeg;                   
extern float deltaHeadingRad;
extern float deltaHeadingNegRad;
extern float cellSize;                          
extern float tieBreaker;                        

// HEURISTIC CONSTANTS
extern float factor2D;
extern float penaltyTurning;
extern float penaltyReversing;
extern float penaltyCOD;
extern float dubinsShotDistance;
extern float dubinsStepSize;

// DUBINS LOOKUP SPECIFIC
extern int dubinsWidth;
extern int dubinsArea;


// _________________________
// COLLISION LOOKUP SPECIFIC

/// [m] -- The bounding box size length and width to precompute all possible headings
extern int bbSize;
/// [#] --- The sqrt of the number of discrete positions per cell
static const int positionResolution = 10;
/// [#] --- The number of discrete positions per cell
static const int positions = positionResolution * positionResolution;

struct relPos {
    int x;
    int y;
};

// cells covered by the vehicle in one pose: 64 in the original, enough for 1 m cells only (the CAT Vehicle on
// 0.5 m cells covers ~100)
static const int maxFootprintCells = 256;
struct config {
    int length;
    relPos pos[maxFootprintCells];
};

// _________________
static const float minRoadWidth = 2;        

struct color {
  float red;
  float green;
  float blue;
};

static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};

extern bool LoadParamFromYaml(const std::string& config_file_path);
}
}

#endif // CONSTANTS

