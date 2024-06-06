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
#include <object.hpp>
#include <cmath>
#include "frameBufferObject.hpp"
#include "modelShader.hpp"

using std::string;

class WaterShader
{
public:
    WaterShader() {}
    WaterShader(string &vsPath, string &fsPath, string &waterPath, ObjectShader objectShader_, ModelShader modelShader_)
    {
        waterShader = Shader(vsPath, fsPath);
        objectShader = objectShader_;
        modelShader = modelShader_;
        createWaterPlane();
        reflectionFBO = FBO(false);
        string test123;
        test123 = fs::path(waterPath) / "waterDUDV.png";
        distortionTexture = Texture(test123, false);
        test123 = fs::path(waterPath) / "waterNormalMap.png";
        normalMap = Texture(test123, false);
    }
    /**
     * @brief Renders water surface with reflection and distortions
     * @param objects Plane objects to be seen in reflections
     * @param model 3D models to be seen in reflections
     * @param camera The camera that is viewing the scene
     * @param renderFBO The FBO that the water will be drawn onto
     */
    void render(std::vector<Object> objects, Model model, Camera camera, FBO renderFBO)
    {
        glEnable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        // Mirror camera accross water plane and render reflected scene
        reflectionFBO.use();
        reflectionFBO.clear();
        camera.flip();
        objectShader.render(objects, camera);
        modelShader.render(model, camera);

        // Move camera back
        camera.flip();
        // Now render the water with the reflection texture
        renderFBO.use();
        waterShader.use();
        // Set uniform variables and things
        waterShader.setMat4("projection", camera.getProjectionMatrix());
        waterShader.setMat4("view", camera.getViewMatrix());
        waterShader.setMat4("model", glm::mat4(1.0f));
        waterShader.setVec3("cameraPos", camera.getPosition());
        waterShader.setInt("reflectionTex", 0);
        reflectionFBO.useTexture(0);
        waterShader.setInt("dudv", 1);
        waterShader.setInt("normal", 2);
        waterShader.setVec4("fogColor", glm::vec4(FOG_COLOR, 1.0));
        float moveFactor = glfwGetTime() * WAVE_SPEED;
        waterShader.setFloat("moveFactor", moveFactor);
        waterShader.setFloat("fogStrength", FOG_STRENGTH);
        waterShader.setFloat("waveDistortion", WAVE_DISTORTION);
        distortionTexture.use(1);
        normalMap.use(2);
        glBindVertexArray(waterVAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }

private:
    void createWaterPlane()
    {
        // Verteex information
        float vertices[20] = {
            // positions                    // distortion coords
            0.5 * POOL_WIDTH, 0.5 * POOL_LENGTH, 0, POOL_WIDTH * 1.0 / WAVE_SCALE, POOL_LENGTH * 1.0 / WAVE_SCALE, // top right
            0.5 * POOL_WIDTH, -0.5 * POOL_LENGTH, 0, POOL_WIDTH * 1.0 / WAVE_SCALE, 0.0,                           // bottom right
            -0.5 * POOL_WIDTH, -0.5 * POOL_LENGTH, 0, 0.0, 0.0,                                                    // bottom left
            -0.5 * POOL_WIDTH, 0.5 * POOL_LENGTH, 0, 0.0, POOL_LENGTH * 1.0 / WAVE_SCALE                           // top left
        };

        // Set the stuffs and things
        glGenVertexArrays(1, &waterVAO);
        glBindVertexArray(waterVAO);
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        unsigned int indices[6] = {
            0, 1, 3, // first triangle
            1, 2, 3  // second triangle
        };
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);
        // texture coord attribute
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }

    //===============================//
    //          VARIABLES            //
    //===============================//
    unsigned int VBO, waterVAO, EBO;
    Texture distortionTexture;
    ObjectShader objectShader;
    ModelShader modelShader;
    Shader waterShader;
    Texture normalMap;
    FBO reflectionFBO;
};