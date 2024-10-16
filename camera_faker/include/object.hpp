#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include <glm/glm.hpp>
#include <stb_image.h>
#include <shader.hpp>
#include <camera.hpp>
#include <iostream>
#include <fstream>
#include <shader.hpp>
#include <texture.hpp>
#include <camera.hpp>
#include <settings.h>
using glm::vec3;
class Object
{
public:
    Object() {}
    Object(std::string imgPath, float width_, float height_, vec3 position_ = vec3(0, 0, 0), vec3 rpy = vec3(0, 0, 0), float textCountW_ = 1, float textCountH_ = 1)
    {
        width = width_;
        height = height_;
        position = position_;
        roll = rpy.x;
        pitch = rpy.y;
        yaw = rpy.z;
        texture = Texture(imgPath);
        textCountH = textCountH_;
        textCountW = textCountW_;

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        generateModelMatrix();
        generateBuffers();
    }
    void use(int textureSlot = 1)
    {
        // Object has image texture in image slot 1
        glBindVertexArray(VAO);
        texture.use(textureSlot);
    }

    /*
     * @breif
     */
    void applyTagOffset(vec3 tagOffset, float tagYaw)
    {
        glm::mat4 tagTransform(1.0);
        tagTransform = glm::rotate(tagTransform, glm::radians(-tagYaw), vec3(0, 0, 1));
        tagTransform = glm::translate(tagTransform, -tagOffset);

        modelMatrix = tagTransform * modelMatrix;
    }
    void updatePosition(vec3 position_)
    {
        position = position_;
        generateModelMatrix();
    }
    void updateOrientation(float pitch_, float roll_, float yaw_)
    {
        pitch = pitch_;
        roll = roll_;
        yaw = yaw_;
        generateModelMatrix();
    }
    glm::mat4 getModelMatrix() const
    {
        return modelMatrix;
    }
    vec3 getPositionXYZ() const
    {
        return position;
    }
    vec3 getOrientationRPY() const
    {
        return vec3(roll * M_PI / 180, pitch * M_PI / 180, (yaw - 90) * M_PI / 180);
    }

private:
    void generateBuffers()
    {
        // Stretch the caustic texture as the plane gets more vertical
        vec3 unitZ(0, 0, 1);
        float stretchW = 1 - CAUSTIC_STRETCH * abs(glm::dot(unitZ, vec3((modelMatrix)[1])));
        float stretchH = 1 - CAUSTIC_STRETCH * abs(glm::dot(unitZ, vec3(modelMatrix[2])));
        // Vertex information
        float vertices[28] = {
            // positions                    // texture coords                   // cautic cords
            0.0f, 0.5f * width, 0.5f * height, textCountW, textCountH, width / (float)CAUSTIC_SCALE * stretchW, height / (float)CAUSTIC_SCALE * stretchH, // top right
            0.0f, 0.5f * width, -0.5f * height, textCountW, 0.0, width / (float)CAUSTIC_SCALE * stretchW, 0.0,                                            // bottom right
            0.0f, -0.5f * width, -0.5f * height, 0.0, 0.0, 0.0, 0.0,                                                                                      // bottom left
            0.0f, -0.5f * width, 0.5f * height, 0.0, textCountH, 0.0, height / (float)CAUSTIC_SCALE * stretchH                                            // top left
        };
        // Indices of triangle vertices
        unsigned int indices[6] = {
            0, 1, 3, // first triangle
            1, 2, 3  // second triangle
        };
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);
        // texture coord attribute
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        // caustic texture coord attribute
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void *)(5 * sizeof(float)));
        glEnableVertexAttribArray(2);
    }

    // Create model matrix (It's a transformation matrix from model coordinates to world)
    void generateModelMatrix()
    {
        modelMatrix = glm::mat4(1.0f);
        // Apply translation
        modelMatrix = glm::translate(modelMatrix, position);
        // Apply Euler angle rotations
        modelMatrix = glm::rotate(modelMatrix, glm::radians(roll), vec3(1, 0, 0));
        modelMatrix = glm::rotate(modelMatrix, glm::radians(pitch), vec3(0, 1, 0));
        modelMatrix = glm::rotate(modelMatrix, glm::radians(yaw), vec3(0, 0, 1));
    }

    float width;
    float height;
    float roll, pitch, yaw;
    unsigned int VBO, VAO, EBO;
    float textCountW, textCountH;

    Texture texture;
    vec3 position;
    glm::mat4 modelMatrix;
};

std::vector<Object> generatePoolObjects(std::filesystem::path textureFolder)
{
    std::filesystem::path poolFolder = textureFolder / "pool";
    std::vector<Object> poolObjects;
    // POOL FLOOR
    //---------------------------------------
    // Made up of 9 objects, 4 corner objects, 4 edge objects, 1 middle object.
    // Edges are 1.5 pool lanes wide

    int laneCountX = round((POOL_LENGTH * 1.0) / LANE_WIDTH) - 3;
    int laneCountY = round((POOL_WIDTH * 1.0) / LANE_WIDTH) - 3;
    // Middle
    string poolTexture = poolFolder / "Pool_Cross.jpg";
    Object poolFloorMiddle(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_WIDTH - 3 * LANE_WIDTH, vec3(0, 0, -POOL_DEPTH), vec3(0, 90, 0), laneCountX, laneCountY);
    // Edges
    poolTexture = poolFolder / "Pool_Black_T.jpg";
    Object poolFloorEdge1(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, LANE_WIDTH * 1.5, vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, 0, -POOL_DEPTH), vec3(0, -90, 0), laneCountX, 1);
    Object poolFloorEdge2(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, LANE_WIDTH * 1.5, vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, 0, -POOL_DEPTH), vec3(0, 90, 0), laneCountX, 1);
    poolTexture = poolFolder / "Pool_Blue_T.jpg";
    Object poolFloorEdge3(poolTexture, LANE_WIDTH * 1.5, POOL_WIDTH - 3 * LANE_WIDTH, vec3(0, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), vec3(0, 90, 0), 1, laneCountY);
    Object poolFloorEdge4(poolTexture, LANE_WIDTH * 1.5, POOL_WIDTH - 3 * LANE_WIDTH, vec3(0, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), vec3(0, 90, -180), 1, laneCountY);
    // Corners
    poolTexture = poolFolder / "Pool_Corner.jpg";
    Object poolFloorCorner1(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), vec3(0, -90, 0));
    Object poolFloorCorner2(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), vec3(0, 90, 0));
    Object poolFloorCorner3(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), vec3(0, -90, 180));
    Object poolFloorCorner4(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), vec3(0, 90, 180));
    poolTexture = textureFolder / "water" / "caustics" / "Caustics_Texture (69].jpg";
    Object poolFloorCorner5(poolTexture, LANE_WIDTH * 0.125, LANE_WIDTH * 0.125, vec3(POOL_WIDTH / 2 - 0.0625 * LANE_WIDTH, POOL_LENGTH / 2 - 0.0625 * LANE_WIDTH, -POOL_DEPTH * 1.01), vec3(0, 90, 0));

    // POOL WALLS
    //-------------------------------------
    // Wall Sides
    poolTexture = poolFolder / "Pool_Wall_Black.jpg";
    Object poolWallSide1(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, vec3(POOL_WIDTH / 2, 0, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(), laneCountX, 1);
    Object poolWallSide2(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, vec3(-POOL_WIDTH / 2, 0, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(), laneCountX, 1);
    poolTexture = poolFolder / "Pool_Wall_Blue.jpg";
    Object poolWallSide3(poolTexture, POOL_WIDTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, vec3(0, -POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, 90), laneCountY, 1);
    Object poolWallSide4(poolTexture, POOL_WIDTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, vec3(0, POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, 90), laneCountY, 1);
    // Wall corners
    poolTexture = poolFolder / "Pool_Wall_Blue_Corner.jpg";
    Object poolWallCorner1(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, -90));
    Object poolWallCorner2(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, -POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, -90));
    Object poolWallCorner3(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, 90));
    Object poolWallCorner4(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, -POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, 90));
    poolTexture = poolFolder / "Pool_Wall_Black_Corner.jpg";
    Object poolWallCorner5(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(POOL_WIDTH / 2, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, 0));
    Object poolWallCorner6(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(-POOL_WIDTH / 2, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, 0));
    Object poolWallCorner7(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(POOL_WIDTH / 2, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, -180));
    Object poolWallCorner8(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, vec3(-POOL_WIDTH / 2, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), vec3(0, 0, -180));

    // Add objects to list and return
    poolObjects.push_back(poolFloorMiddle);
    poolObjects.push_back(poolFloorEdge1);
    poolObjects.push_back(poolFloorEdge2);
    poolObjects.push_back(poolFloorEdge3);
    poolObjects.push_back(poolFloorEdge4);
    poolObjects.push_back(poolFloorCorner1);
    poolObjects.push_back(poolFloorCorner2);
    poolObjects.push_back(poolFloorCorner3);
    poolObjects.push_back(poolFloorCorner4);
    poolObjects.push_back(poolFloorCorner5);
    poolObjects.push_back(poolWallSide1);
    poolObjects.push_back(poolWallSide2);
    poolObjects.push_back(poolWallSide3);
    poolObjects.push_back(poolWallSide4);
    poolObjects.push_back(poolWallCorner1);
    poolObjects.push_back(poolWallCorner2);
    poolObjects.push_back(poolWallCorner3);
    poolObjects.push_back(poolWallCorner4);
    poolObjects.push_back(poolWallCorner5);
    poolObjects.push_back(poolWallCorner6);
    poolObjects.push_back(poolWallCorner7);
    poolObjects.push_back(poolWallCorner8);
    return poolObjects;
}