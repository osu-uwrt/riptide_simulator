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
#include <object.hpp>
#include <cmath>
#include <causticsManager.hpp>

using std::string;

enum ClippingMode
{
    NO_CLIPPING,
    DISCARD_ABOVE_WATER,
    DISCARD_BELOW_WATER
};
class ObjectShader
{
public:
    ObjectShader() {}
    ObjectShader(string vsPath, string fsPath)
    {
        shader = Shader(vsPath, fsPath);
    }
    /**
     * @brief Draws a plane object onto the currently active FBO.
     * @param objects vector containing all the objects that will be drawn
     * @param camera the camera that is being used to look at the objects
     */
    void render(std::vector<Object> &objects, Camera camera, ClippingMode clippingMode = NO_CLIPPING)
    {
        // Activate shader
        shader.use();
        glEnable(GL_DEPTH_TEST);
        //  Get camera matrixs
        shader.setMat4("projection", camera.getProjectionMatrix());
        shader.setMat4("view", camera.getViewMatrix());

        // Set uniform variables that are constant for all objects
        shader.setVec3("cameraPos", camera.getPosition());
        shader.setFloat("fogStrength", FOG_STRENGTH);
        shader.setFloat("causticStrength", CAUSTIC_STRENGTH);
        shader.setVec4("fogColor", glm::vec4(FOG_COLOR, 1.0));
        shader.setInt("objectTexture", 1);
        shader.setInt("causticTexture", 2);
        CausticsManager::setTextureSlot(2);
        shader.setInt("clippingMode", clippingMode);
        // Loop through each object to render
        for (Object &object : objects)
        {
            object.use(1);
            shader.setMat4("model", object.getModelMatrix());
            // Draw two triangles on the screen
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }
    }

private:
    Shader shader;
};
