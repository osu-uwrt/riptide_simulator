
#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <settings.h>
#include <vector>
#include <algorithm>

using std::cout, std::endl;

enum CameraMode
{
    ROBOT_MODE,
    FLY_ARROUND_MODE
};

// Camera movement options
enum Camera_Movement
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
    // constructor with vectors
    Camera(glm::vec3 position_ = glm::vec3(0.0f, 0.0f, -0.7f))
    {
        position = position_;
        eulerAngles = glm::vec3(0, 0, 0);
    }
    glm::mat4 getViewMatrix()
    {
        // Get some vectors about where the camera is looking from the rotation matrix
        glm::mat3 rotM = rpy2rotM(eulerAngles);
        glm::vec3 front = glm::normalize(rotM[0]);
        glm::vec3 up = glm::normalize(rotM[2]);

        // Return the view matrix, maps from world coordinates -> camera coordinates
        return glm::lookAt(position, position + front, up);
    }
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        // Get some vectors about where the camera is looking from the rotation matrix
        glm::mat3 rotM = rpy2rotM(eulerAngles);
        glm::vec3 front = rotM[0];
        glm::vec3 right = rotM[1];

        // Move the camera position based on the desired direction
        float velocity = MOVEMENT_SPEED * deltaTime;
        if (direction == FORWARD)
            position += glm::normalize(glm::vec3(front.x, front.y, 0)) * velocity;
        if (direction == BACKWARD)
            position -= glm::normalize(glm::vec3(front.x, front.y, 0)) * velocity;
        if (direction == LEFT)
            position += glm::normalize(glm::vec3(right.x, right.y, 0)) * velocity;
        if (direction == RIGHT)
            position -= glm::normalize(glm::vec3(right.x, right.y, 0)) * velocity;
        if (direction == UP)
            position += glm::vec3(0, 0, 1) * velocity;
        if (direction == DOWN)
            position -= glm::vec3(0, 0, 1) * velocity;
    }
    // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true)
    {
        xoffset *= MOUSE_SENSITIVITY;
        yoffset *= MOUSE_SENSITIVITY;

        eulerAngles.x += xoffset; // yaw
        eulerAngles.y -= yoffset; // pitch
        eulerAngles.z = 0;        // roll

        // Clamping pitch to +/- 90 degrees
        eulerAngles.y = std::clamp(eulerAngles.y, -(float)M_PI / 2, (float)M_PI / 2);
    }

    // Flips camera position and orientation about water surface
    // Used for rendering reflections
    void flip()
    {
        position.z = -position.z;       // Flip z position
        eulerAngles.y = -eulerAngles.y; // Flip pitch
        eulerAngles.z = -eulerAngles.z; // Flip roll
    }
    void setYPR(float y, float p, float r)
    {
        eulerAngles.x = y;
        eulerAngles.y = p;
        eulerAngles.z = r;
    }
    void setPosition(float x, float y, float z)
    {
        position = glm::vec3(x, y, z);
    }
    void setPosition(glm::vec3 position_)
    {
        position = position_;
    }
    glm::vec3 getPosition()
    {
        return position;
    }

private:
    // Converts a vector holding roll, pitch, yaw into a 3x3 rotation matrix
    glm::mat3 rpy2rotM(glm::vec3 ypr)
    {
        glm::mat4 rotM = glm::mat4(1.0f);
        // Apply Euler angle rotations
        rotM = glm::rotate(rotM, ypr.x, glm::vec3(0, 0, 1)); // yaw
        rotM = glm::rotate(rotM, ypr.y, glm::vec3(0, 1, 0)); // pitch
        rotM = glm::rotate(rotM, ypr.z, glm::vec3(1, 0, 0)); // roll
        return glm::mat3(rotM);
    }

    glm::vec3 position;
    glm::vec3 eulerAngles;
};
