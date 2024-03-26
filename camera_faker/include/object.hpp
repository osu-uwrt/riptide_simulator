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
#include <light.hpp>
#include <material.hpp>
namespace fs = std::filesystem;
class Object
{
public:
    Object() {}
    Object(std::string imgPath, float width_, float height_, glm::vec3 position_ = glm::vec3(0, 0, 0), float roll_ = 0, float pitch_ = 0, float yaw_ = 0, float textCountW_ = 1, float textCountH_ = 1)
    {
        width = width_;
        height = height_;
        position = position_;
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
        texture = Texture(imgPath);
        textCountH = textCountH_;
        textCountW = textCountW_;

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        generateBuffers();
        generateModelMatrix();
    }
    void use()
    {
        // Object has image texture in image slot 1
        glBindVertexArray(VAO);
        texture.use(1);
    }
    void updatePosition(glm::vec3 position_)
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
    glm::mat4 getModelMatrix()
    {
        return modelMatrix;
    }

private:
    void generateBuffers()
    {
        // Verteex information
        float vertices[28] = {
            // positions                    // texture coords                   // cautic cords
            0.5f * width, 0.0, 0.5f * height, textCountW, textCountH, width / (float)CAUSTIC_SCALE, height / (float)CAUSTIC_SCALE, // top right
            0.5f * width, 0.0, -0.5f * height, textCountW, 0.0, width / (float)CAUSTIC_SCALE, 0.0,                                 // bottom right
            -0.5f * width, 0.0, -0.5f * height, 0.0, 0.0, 0.0, 0.0,                                                                // bottom left
            -0.5f * width, 0.0, 0.5f * height, 0.0, textCountH, 0.0, height / (float)CAUSTIC_SCALE                                 // top left
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
        modelMatrix = glm::rotate(modelMatrix, glm::radians(roll), glm::vec3(1, 0, 0));
        modelMatrix = glm::rotate(modelMatrix, glm::radians(pitch), glm::vec3(0, 1, 0));
        modelMatrix = glm::rotate(modelMatrix, glm::radians(yaw), glm::vec3(0, 0, 1));
    }

    float width;
    float height;
    float roll, pitch, yaw;
    unsigned int VBO, VAO, EBO;
    float textCountW, textCountH;

    Texture texture;
    Material material;
    glm::vec3 position;
    glm::mat4 modelMatrix;
};

std::vector<Object> generatePoolObjects(string textureFolder)
{
    fs::path poolFolder = fs::path(textureFolder) / "pool";
    std::vector<Object> poolObjects;
    // POOL FLOOR
    //---------------------------------------
    // Made up of 9 objects, 4 corner objects, 4 side objects, 1 middle object.
    // Middle
    string poolTexture = poolFolder / "Pool_Cross.jpg";
    Object poolFloorMiddle(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_WIDTH - 3 * LANE_WIDTH, glm::vec3(0, 0, -POOL_DEPTH), 0, -90, -90, 17, 7);
    // Edges
    poolTexture = poolFolder / "Pool_Black_T.jpg";
    Object poolFloorEdge1(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, LANE_WIDTH * 1.5, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, 0, -POOL_DEPTH), 0, -90, -90, 17, 1);
    Object poolFloorEdge2(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, LANE_WIDTH * 1.5, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, 0, -POOL_DEPTH), 0, 90, -90, 17, 1);
    poolTexture = poolFolder / "Pool_Blue_T.jpg";
    Object poolFloorEdge3(poolTexture, LANE_WIDTH * 1.5, POOL_WIDTH - 3 * LANE_WIDTH, glm::vec3(0, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, 90, 1, 7);
    Object poolFloorEdge4(poolTexture, LANE_WIDTH * 1.5, POOL_WIDTH - 3 * LANE_WIDTH, glm::vec3(0, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, -90, 1, 7);
    // Corners
    poolTexture = poolFolder / "Pool_Corner.jpg";
    Object poolFloorCorner1(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, -90, 90, 1, 1);
    Object poolFloorCorner2(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, 90, 1, 1);
    Object poolFloorCorner3(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, -90, -90, 1, 1);
    Object poolFloorCorner4(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, -90, 1, 1);

    // POOL WALLS
    //-------------------------------------
    // Wall Sides
    poolTexture = poolFolder / "Pool_Wall_Black.jpg";
    Object poolWallSide1(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(POOL_WIDTH / 2, 0, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 90, 17, 1);
    Object poolWallSide2(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(-POOL_WIDTH / 2, 0, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 90, 17, 1);
    poolTexture = poolFolder / "Pool_Wall_Blue.jpg";
    Object poolWallSide3(poolTexture, POOL_WIDTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(0, -POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 0, 7, 1);
    Object poolWallSide4(poolTexture, POOL_WIDTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(0, POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 0, 7, 1);
    // Wall corners
    poolTexture = poolFolder / "Pool_Wall_Blue_Corner.jpg";
    Object poolWallCorner1(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 0);
    Object poolWallCorner2(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, -POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 0);
    Object poolWallCorner3(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 180);
    Object poolWallCorner4(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, -POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 180);
    poolTexture = poolFolder / "Pool_Wall_Black_Corner.jpg";
    Object poolWallCorner5(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(POOL_WIDTH / 2, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 90);
    Object poolWallCorner6(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(-POOL_WIDTH / 2, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 90);
    Object poolWallCorner7(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(POOL_WIDTH / 2, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, -90);
    Object poolWallCorner8(poolTexture, LANE_WIDTH * 1.5, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(-POOL_WIDTH / 2, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, -90);

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