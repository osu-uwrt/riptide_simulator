#pragma once
//===============================//
//     SETTINGS/CONSTANTS        //
//===============================//

#define ACOUSTIC_DATA true       // Toggle whether acoustic sensor data will be faked
#define COLLISION_TOGGLE true     // Toggle whether collisions are enabled or disabled
#define STATE_PUB_TIME 0.01       // Time, in seconds, between simulator publishing state (for RViz)
#define COEF_OF_RESTITUTION 0.3   // Ratio of velocity after vs before collision https://en.wikipedia.org/wiki/Coefficient_of_restitution
#define PHYSICS_STEP 0.002        // s   fixed integration step (500 Hz)
#define MAX_CATCHUP_STEPS 20      // prevents an overloaded simulator from spiraling behind wall time
#define GRAVITY 9.80665           // m/s^2
