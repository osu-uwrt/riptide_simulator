#pragma once
#include "rclcpp/rclcpp.hpp"
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
#include <light.hpp>
#include <object.hpp>
#include <cmath>
#include <frame_buffer_object.hpp>

class WaterShader
{
public:
    WaterShader() {}
    WaterShader(const char *vsPath, const char *fsPath)
    {
        waterShader = Shader(vsPath, fsPath);
        createWaterPlane();
        reflectionFBO = FBO(true);
    }
    void render(std::vector<Object> objects, Camera camera, FBO renderFBO, ObjectShader objectShader)
    {
        // FBO Activate
        // Move camera
        // Render scene
        // Default FBO activate
        // Move camera back
        // water shader active
        // Render water

        // Move the camera to render the reflection
        // reflectionFBO.use();
        // reflectionFBO.clear();
        camera.Position = glm::vec3(camera.Position.x, camera.Position.y, -camera.Position.z);
        camera.Pitch = -camera.Pitch;
        camera.updateCameraVectors();
        objectShader.render(objects, camera);

        // Move camera back
        camera.Pitch = -camera.Pitch;
        camera.Position = glm::vec3(camera.Position.x, camera.Position.y, -camera.Position.z);
        camera.updateCameraVectors();
        // Now render the water with the texture
        // renderFBO.use();
        waterShader.use();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)IMG_WIDTH / (float)IMG_HEIGHT, 0.1f, 100.0f);
        waterShader.setMat4("projection", projection);
        waterShader.setMat4("view", camera.GetViewMatrix());
        waterShader.setMat4("model", glm::mat4(1.0f));
        waterShader.setVec3("cameraPos", camera.Position);
        waterShader.setInt("reflectionTex", 0);
        waterShader.setInt("dudv", 1);
        waterShader.setInt("normal", 2);
        float moveFactor = glfwGetTime() * WAVE_SPEED;
        waterShader.setFloat("moveFactor", moveFactor);
        glActiveTexture(GL_TEXTURE0);
        // glBindTexture(GL_TEXTURE_2D, reflectionFBO.textureID());
        distortionTexture.use(1);
        normalMap.use(2);
        glBindVertexArray(waterVAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }

private:
    void createWaterPlane()
    {
        // Verteex information
        float vertices[28] = {
            // positions                    // distortion coords
            0.5 * POOL_LENGTH, 0.5 * POOL_WIDTH, 0, POOL_LENGTH * 1.0 / WAVE_SCALE, POOL_WIDTH * 1.0 / WAVE_SCALE, // top right
            0.5 * POOL_LENGTH, -0.5 * POOL_WIDTH, 0, POOL_LENGTH * 1.0 / WAVE_SCALE, 0.0,                          // bottom right
            -0.5 * POOL_LENGTH, 0.5 * POOL_WIDTH, 0, 0.0, 0.0,                                                     // bottom left
            -0.5 * POOL_LENGTH, -0.5 * POOL_WIDTH, 0, 0.0, POOL_WIDTH * 1.0 / WAVE_SCALE                           // top left
        };

        // Set the stuffs and things
        glBindVertexArray(waterVAO);
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

    unsigned int VBO, waterVAO, EBO;
    unsigned int indices[6] = {
        0, 1, 3, // first triangle
        1, 2, 3  // second triangle
    };

    FBO reflectionFBO;
    Texture distortionTexture;
    Texture normalMap;
    Shader waterShader;
};