
#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <settings.h>
#include <vector>
#include <algorithm>

using std::cout, std::endl, glm::vec3;

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
    Camera(vec3 position = vec3(0.0f, 0.0f, -0.7f), float nearPlane = 0.05f, float farPlane = 100.0f)
        : position(position), nearPlane(nearPlane), farPlane(farPlane)
    {
        setIntrinsicParameters(CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY, IMG_WIDTH, IMG_HEIGHT);
    }

    void setIntrinsicParameters(float fx, float fy, float cx, float cy, int width, int height)
    {
        this->fx = fx;
        this->fy = fy;
        this->cx = cx;
        this->cy = cy;
        this->width = width;
        this->height = height;
        updateProjectionMatrix();
    }

    void updateProjectionMatrix()
    {
        float aspectRatio = static_cast<float>(width) / static_cast<float>(height);
        float fovY = 2.0f * glm::atan(0.5f * static_cast<float>(height) / fy);
        projectionMatrix = glm::perspective(fovY, aspectRatio, nearPlane, farPlane);
    }

    glm::mat4 getProjectionMatrix()
    {
        return projectionMatrix;
    }

    glm::mat4 getViewMatrix()
    {
        // Get some vectors about where the camera is looking from the rotation matrix
        glm::mat3 rotM = rpy2rotM(eulerAnglesYPR);
        vec3 front = glm::normalize(rotM[0]);
        vec3 up = glm::normalize(rotM[2]);

        // Return the view matrix, maps from world coordinates -> camera coordinates
        return glm::lookAt(position, position + front, up);
    }
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        // Get some vectors about where the camera is looking from the rotation matrix
        glm::mat3 rotM = rpy2rotM(eulerAnglesYPR);
        vec3 front = rotM[0];
        vec3 right = rotM[1];

        // Move the camera position based on the desired direction
        float velocity = MOVEMENT_SPEED * deltaTime;
        if (direction == FORWARD)
            position += glm::normalize(vec3(front.x, front.y, 0)) * velocity;
        if (direction == BACKWARD)
            position -= glm::normalize(vec3(front.x, front.y, 0)) * velocity;
        if (direction == LEFT)
            position += glm::normalize(vec3(right.x, right.y, 0)) * velocity;
        if (direction == RIGHT)
            position -= glm::normalize(vec3(right.x, right.y, 0)) * velocity;
        if (direction == UP)
            position += vec3(0, 0, 1) * velocity;
        if (direction == DOWN)
            position -= vec3(0, 0, 1) * velocity;
    }
    // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset)
    {
        xoffset *= MOUSE_SENSITIVITY;
        yoffset *= MOUSE_SENSITIVITY;

        eulerAnglesYPR.x += xoffset; // yaw
        eulerAnglesYPR.y -= yoffset; // pitch
        eulerAnglesYPR.z = 0;        // roll

        // Clamping pitch to +/- 90 degrees
        eulerAnglesYPR.y = std::clamp(eulerAnglesYPR.y, -(float)M_PI / 2, (float)M_PI / 2);
    }

    /**
     * @brief Flips camera position and orientation about water surface
     * Used for rendering reflections
     */
    void flip()
    {
        position.z = -position.z;             // Flip z position
        eulerAnglesYPR.y = -eulerAnglesYPR.y; // Flip pitch
        eulerAnglesYPR.z = -eulerAnglesYPR.z; // Flip roll
    }
    void setYPR(float y, float p, float r)
    {
        eulerAnglesYPR.x = y;
        eulerAnglesYPR.y = p;
        eulerAnglesYPR.z = r;
    }
    void setPosition(float x, float y, float z)
    {
        position = vec3(x, y, z);
    }
    void setPosition(vec3 position_)
    {
        position = position_;
    }
    vec3 getPosition()
    {
        return position;
    }

private:
    vec3 position;
    vec3 eulerAnglesYPR = vec3(0, 0, 0);
    glm::mat4 projectionMatrix;
    float fx, fy, cx, cy;      // Intrinsic parameters
    int width, height;         // Image dimensions
    float nearPlane, farPlane; // Clipping planes

    // Converts a vector holding roll, pitch, yaw into a 3x3 rotation matrix
    glm::mat3 rpy2rotM(vec3 ypr)
    {
        glm::mat4 rotM = glm::mat4(1.0f);
        // Apply Euler angle rotations
        rotM = glm::rotate(rotM, ypr.x, vec3(0, 0, 1)); // yaw
        rotM = glm::rotate(rotM, ypr.y, vec3(0, 1, 0)); // pitch
        rotM = glm::rotate(rotM, ypr.z, vec3(1, 0, 0)); // roll
        return glm::mat3(rotM);
    }
};
