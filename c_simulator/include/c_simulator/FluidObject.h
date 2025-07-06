#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>

typedef Eigen::Vector3d v3d;



//define the fluid to add a slight current into the water
//currently based on flow caused by a doublet

class FluidObject{

    FluidObject(v3d InitialSourcePosition, v3d InitialSinkPosition, double InitialStrength, double MaxStrength, double MaxWaterVelocity);

    v3d getFluidVelocity(double XPos, double YPos, double ZPos);
    void cycle(double CurrentTime); // update the doublet with time

    v3d source_location; // the location of the doublet source
    v3d sink_location; // the location of the doublet sink
    double feature_strength; // the strength of the doublet

    double last_update_time;// the last time cycle was called

    double max_feature_strength; // the highest allowable source sink strength
    double source_radius; //the radius of the source and sink 
};