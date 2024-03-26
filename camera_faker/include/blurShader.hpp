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
#include <filesystem>
#include <shader.hpp>
#include <texture.hpp>
#include <camera.hpp>
#include <light.hpp>
#include <object.hpp>
#include <cmath>
#include <frame_buffer_object.hpp>
namespace fs = std::filesystem;
using std::string, std::cout, std::endl;

class BlurShader
{
public:
    BlurShader() {}
    BlurShader(string vsPath, string fsPath)
    {
        blurShader = Shader(vsPath, fsPath);
        horizontalFBO = FBO(false);
        verticalFBO = FBO(false);
        createVertexInfo();
    }

    // Render texture buffer on whole screen
    void renderBlurred(FBO &frame, FBO &drawOn)
    {
        // Setup
        horizontalFBO.clear();
        verticalFBO.clear();
        blurShader.use();
        glDisable(GL_DEPTH_TEST);
        blurShader.setInt("image", 0);
        glBindVertexArray(VAO);
        blurShader.setInt("depthTexture", 1);
        frame.useTexture(1, true);
        // Blur for the desired iterations, ping ponging back and forth between different FBOs
        // Blurring over several iterations with a smaller kernel is more effecient that a large kernel because much less sampling is required
        // Also, doing horizontal and vertical blurs is more effecient again because less sampling
        for (int i = 0; i < BLUR_ITERATIONS; i++)
        {
            // Blur horizontaly
            blurShader.setBool("horizontal", true);
            horizontalFBO.use();
            if (i == 0)
                frame.useTexture(0); // For first run of loop, there hasn't been a vertical blur yet so blur starting frame
            else
                verticalFBO.useTexture(0);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
            // Blur vertically
            blurShader.setBool("horizontal", false);
            if (i == (int)BLUR_ITERATIONS - 1)
            {
                drawOn.use(); // At the last loop iteration, don't want to draw onto vertical FBO anymore, draw onto desired one
            }
            else
                verticalFBO.use();
            horizontalFBO.useTexture(0);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }
    }

private:
    void createVertexInfo()
    {
        //===============================//
        //      SIMPLE SCREEN QUAD       //
        //===============================//
        // Verteex information
        float vertices[16] = {
            // Screen position  // texture coordinates
            1.0, 1.0, 1.0, 1.0,   // top right
            1.0, -1.0, 1.0, 0.0,  // bottom right
            -1.0, -1.0, 0.0, 0.0, // bottom left
            -1.0, 1.0, 0.0, 1.0   // top left
        };
        // Indices of triangle vertices
        unsigned int indices[6] = {
            0, 1, 3, // first triangle
            1, 2, 3  // second triangle
        };
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);
        // texture coord attribute
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }
    unsigned int VBO, VAO, EBO;
    FBO horizontalFBO;
    FBO verticalFBO;
    Shader blurShader;
};
