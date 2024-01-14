#pragma once
//===============================//
//     SETTINGS/CONSTANTS        //
//===============================//

#define SENSOR_NOISE_ENABLED false // Noise is added to sensor data when true
#define STATE_PUB_TIME 0.02        // Time, in seconds, between simulator publishing state (for RViz)
#define COEF_OF_RESTITUTION 1.0    // Ratio of velocity after vs before collision https://en.wikipedia.org/wiki/Coefficient_of_restitution
#define THRUSTER_DELAY 0.1         // s         time from when thruster is commanded to when force is applied
#define GRAVITY 9.80665            // m/s^2
#define VEHICLE_HEIGHT 0.3         // m         Used for bouyancy calcs when partially submerged
