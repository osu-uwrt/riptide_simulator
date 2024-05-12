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

class FrameShader
{
public:
    FrameShader() {}
    FrameShader(string vsPath, string fsPath)
    {
        shader = Shader(vsPath, fsPath);
        createVertexInfo();
    }
    // Render distorted texture on whole screen
    void renderDistorted(FBO &frame)
    {
        shader.use();
        glDisable(GL_DEPTH_TEST);
        shader.setInt("ourTexture", 0);
        frame.useTexture(0);
        shader.setInt("depthTexture", 1);
        frame.useTexture(1, true);
        glBindVertexArray(distVAO);
        glDrawElements(GL_TRIANGLES, 3 * triangleCount, GL_UNSIGNED_INT, 0);
    }
    // Render texture buffer on whole screen
    void render(FBO &frame)
    {
        shader.use();
        glDisable(GL_DEPTH_TEST);
        shader.setInt("ourTexture", 0);
        frame.useTexture(0);
        shader.setInt("depthTexture", 1);
        frame.useTexture(1, true);
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }
    // Render texture on whole screen
    void render(Texture &texture)
    {
        shader.use();
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_DEPTH_TEST);
        shader.setInt("ourTexture", 0);
        texture.use(0);
        shader.setInt("depthTexture", 1);
        texture.use(1);
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }

private:
    void createVertexInfo()
    {
        // glGenBuffers(1, &distVBO);
        // glGenBuffers(1, &distVAO);
        // glGenBuffers(1, &distEBO);

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

        //===============================//
        //       DISTORTION MESH         //
        //===============================//

        int gridSize = DIST_REFINEMENT;
        float distVertices[gridSize * gridSize * 4];
        // Create grid of points
        for (int j = 0; j < gridSize; j++)
        {
            for (int i = 0; i < gridSize; i++)
            {
                // Calculate normalized screen coordinates (range from -1 to 1)
                float x = (i / (gridSize - 1.0f) * 2.0f - 1.0f);
                float y = (1.0f - j / (gridSize - 1.0f) * 2.0f);

                // Convert to normalized camera coordinates
                float xc = x * CAMERA_FX + CAMERA_CX;
                float yc = y * CAMERA_FY + CAMERA_CY;

                // Calculate radial distance from the center of the image
                float r2 = xc * xc + yc * yc;

                // Apply radial and tangential distortion
                float xDistorted = xc * (1 + DIST_K1 * r2 + DIST_K2 * r2 * r2 + DIST_K3 * r2 * r2 * r2) +
                                2 * DIST_P1 * xc * yc + DIST_P2 * (r2 + 2 * xc * xc);
                float yDistorted = yc * (1 + DIST_K1 * r2 + DIST_K2 * r2 * r2 + DIST_K3 * r2 * r2 * r2) +
                                DIST_P1 * (r2 + 2 * yc * yc) + 2 * DIST_P2 * xc * yc;

                // Convert back to OpenGL screen coordinates
                float xGL = (xDistorted - CAMERA_CX) / CAMERA_FX;
                float yGL = (yDistorted - CAMERA_CY) / CAMERA_FY;

                // Store the distorted vertex coordinates
                int index = (j * gridSize + i) * 4;
                distVertices[index] = xGL;
                distVertices[index + 1] = yGL;
                distVertices[index + 2] = i / (gridSize - 1.0f);
                distVertices[index + 3] = j / (gridSize - 1.0f);
            }
        }
        //  Create indices array
        triangleCount = pow((gridSize - 1), 2) * 2;
        unsigned int distIndices[3 * triangleCount];
        /* Loop through all squares in grid and split into two triangles
        Example indices shown for 4x4 grid
                 +i->
            0,  1,  2,  3
       +j|  4,  5,  6,  7
         v  8,  9,  10, 11
            12, 13, 14, 15
        */
        int triangleCounter = 0;
        for (int i = 0; i < gridSize - 1; i++)
        {
            for (int j = 0; j < gridSize - 1; j++)
            {
                int startIndex = gridSize * j + i;
                /* First triangle
                    0__1
                    | /
                    |/
                    2
                */
                distIndices[3 * triangleCounter + 0] = startIndex;
                distIndices[3 * triangleCounter + 1] = startIndex + 1;
                distIndices[3 * triangleCounter + 2] = startIndex + gridSize;
                triangleCounter++;
                /* Second triangle
                      2
                     /|
                    /_|
                   1   0
                */
                distIndices[3 * triangleCounter + 0] = startIndex + gridSize + 1;
                distIndices[3 * triangleCounter + 1] = startIndex + gridSize;
                distIndices[3 * triangleCounter + 2] = startIndex + 1;
                triangleCounter++;
            }
        }
        glGenVertexArrays(1, &distVAO);
        glBindVertexArray(distVAO);
        glGenBuffers(1, &distVBO);
        glBindBuffer(GL_ARRAY_BUFFER, distVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(distVertices), distVertices, GL_STATIC_DRAW);
        glGenBuffers(1, &distEBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, distEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(distIndices), distIndices, GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);
        // texture coord attribute
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }

    //===============================//
    //          VARIABLES            //
    //===============================//
    unsigned int triangleCount;
    unsigned int VBO, VAO, EBO;
    unsigned int distVBO, distVAO, distEBO;
    Shader shader;
};
