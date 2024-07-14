#include "c_simulator/FluidOject.h"

typedef Eigen::Vector3d v3d;


FluidObject::FluidOject(v3d InitialSourcePosition, v3d InitialSinkPosition, double InitialStrength, double MaxStrength, double sourceRadius){
    this->source_location = InitialSourcePosition;
    this->sink_location = InitialSinkPosition;
    this->feature_strength = InitialStrength;
    this->max_feature_strength = MaxStrength;
    
    //don't want a current velocity over Max Water Velocity
    this-> sourceRadius = sourceRadius;
}


run()
(Q ((x cos(t) - c)^2 + (y sin(t) - d)^2) ((2 cos(t) (x cos(t) - a) + 2 sin(t) (x sin(t) - b))/((x cos(t) - c)^2 + (y sin(t) - d)^2) - (2 cos(t) (x cos(t) - c) ((x cos(t) - a)^2 + (x sin(t) - b)^2))/((x cos(t) - c)^2 + (y sin(t) - d)^2)^2))/(4 π ((x cos(t) - a)^2 + (x sin(t) - b)^2))