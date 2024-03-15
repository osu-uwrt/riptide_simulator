//===============================//
//      POOL SETTINGS            //
//===============================//
// Note that water height is at z = 0 m
#define POOL_LENGTH 50.0         // m
#define POOL_WIDTH 25.0          // m
#define LANE_WIDTH 2.77777777778 // m   it's a funky number but that's what the pool was
#define POOL_DEPTH 2.1336        // m   or 7 ft
#define LEDGE_HEIGHT 0.305288888 // m   or 1 ft How high above the water level the concrete is

//===============================//
//      MOVEMENT SETTINGS        //
//===============================//
#define MOUSE_SENSITIVITY 0.15 // Lower number makes fly around camera rotate slower
#define MOVEMENT_SPEED 5.0     // m/s translation speed of fly around camera

//===============================//
//        CAMERA SETTINGS        //
//===============================//
#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080
#define FRAME_RATE 30                // FPS data is published at, will still render at max speed
#define CAMERA_CX 619.707            // Intrinsic camera parameter in projection matrix
#define CAMERA_CY 360.589            // Intrinsic camera parameter in projection matrix
#define CAMERA_FX 707.696            // Intrinsic camera parameter in projection matrix
#define CAMERA_FY 707.254            // Intrinsic camera parameter in projection matrix
#define DIST_K1 0.1816198718936867   // Radial camera distortion constant
#define DIST_K2 0.4879032297013213   // Radial camera distortion constant
#define DIST_K3 -0.438187135448847   // Radial camera distortion constant
#define DIST_P1 0.005421094419821622 // Tangential camera distortion constant
#define DIST_P2 -0.01686702929877146 // Tangential camera distortion constant
#define MOTION_BLUR_FRAMES 2         // Number of previous frames used for motion blurring, set to zero to disable
#define BLUR_RADIUS 2                // Pixel radius of Gaussian blur kernel, set to zero to disable blur

//===============================//
//         LIGHT SETTINGS        //
//===============================//

//===============================//
//         FOG SETTINGS          //
//===============================//
#define FOG_STRENGTH 0.15                // [0.0-1.0] High number represents stronger fog
#define FOG_COLOR 0.004f, 0.552f, 0.645f // R,G,B values [0.0-1.0]

//===============================//
//     CAUSTIC SETTINGS          //
//===============================//
#define CAUSTIC_STRENGTH 0.5 // [0.0-1.0] High number represets stronger caustics
#define CAUSTIC_SPEED 20.5   // Speed of caustic image animation
#define CAUSTIC_SCALE 3.0    // m  - size of caustic texture square

//===============================//
//         WATER SETTINGS        //
//===============================//
#define WAVE_SPEED 0.1      // Controls how fast the water waves move
#define WAVE_STRENGTH 0.005 // Controls how much distortion the waves create
#define WAVE_SCALE 0.1      // Controls the size of the waves