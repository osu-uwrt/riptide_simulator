#pragma once
//===============================//
//     SETTINGS/CONSTANTS        //
//===============================//

/** @note If SYNC_ODOM is set to true, fake sensor data is ignored */
#define SYNC_ODOM true            // Makes it so Odometry position perfically matches physics position
#define SENSOR_NOISE_ENABLED true // Noise is added to sensor data when true
#define ACOUSTIC_DATA false       // Toggle whether acoustic sensor data will be faked
#define COLLISION_TOGGLE true     // Toggle whether collisions are enabled or disabled
#define STATE_PUB_TIME 0.01       // Time, in seconds, between simulator publishing state (for RViz)
#define COEF_OF_RESTITUTION 0.3   // Ratio of velocity after vs before collision https://en.wikipedia.org/wiki/Coefficient_of_restitution
#define THRUSTER_DELAY 0.1        // s   time from when thruster is commanded to when force is applied
#define GRAVITY 9.80665           // m/s^2
#define VEHICLE_HEIGHT 0.3        // m   Used for bouyancy calcs when partially submerged
