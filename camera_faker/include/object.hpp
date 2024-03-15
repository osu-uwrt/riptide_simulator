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
            0.5 * width, 0.0, 0.5 * height, textCountW, textCountH, width / CAUSTIC_SCALE, height / CAUSTIC_SCALE, // top right
            0.5 * width, 0.0, -0.5 * height, textCountW, 0.0, width / CAUSTIC_SCALE, 0.0,                          // bottom right
            -0.5 * width, 0.0, -0.5 * height, 0.0, 0.0, 0.0, 0.0,                                                  // bottom left
            -0.5 * width, 0.0, 0.5 * height, 0.0, textCountH, 0.0, height / CAUSTIC_SCALE                          // top left
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

    void generateModelMatrix()
    {
        modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::translate(modelMatrix, position);
        modelMatrix = glm::rotate(modelMatrix, glm::radians(roll), glm::vec3(1, 0, 0));
        modelMatrix = glm::rotate(modelMatrix, glm::radians(pitch), glm::vec3(0, 1, 0));
        modelMatrix = glm::rotate(modelMatrix, glm::radians(yaw), glm::vec3(0, 0, 1));
    }

    float width;
    float height;
    float roll, pitch, yaw;
    float textCountW, textCountH;
    unsigned int VBO, VAO, EBO;
    unsigned int indices[6] = {
        0, 1, 3, // first triangle
        1, 2, 3  // second triangle
    };

    Texture texture;
    Material material;
    glm::mat4 modelMatrix;
    glm::vec3 position;
};