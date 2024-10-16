#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include "shader.hpp"
#include "camera.hpp"
#include "objectShader.hpp"
#include <cmath>
#include "model.hpp"
#include "causticsManager.hpp"

namespace fs = std::filesystem;
using std::string;

class ModelShader
{
public:
    ModelShader() {}
    ModelShader(string vsPath, string fsPath)
    {
        shader = Shader(vsPath, fsPath);
    }

    /**
     * @brief Draws a 3D model onto the currently active FBO.
     * @param models Vector of 3D model that will be drawn
     * @param camera the camera that is being used to look at the model
     */
    void render(std::vector<Model> &models, Camera camera, ClippingMode clippingMode = NO_CLIPPING)
    {
        // Activate shader
        shader.use();
        glEnable(GL_DEPTH_TEST);
        // Get camera matrixs
        shader.setMat4("view", camera.getViewMatrix());
        shader.setMat4("projection", camera.getProjectionMatrix());

        // Set uniform variables that are constant for all objects
        shader.setVec3("cameraPos", camera.getPosition());
        shader.setFloat("fogStrength", FOG_STRENGTH);
        shader.setVec4("fogColor", glm::vec4(FOG_COLOR, 1.0));
        shader.setInt("causticTexture", 2);
        shader.setFloat("causticStrength", CAUSTIC_STRENGTH);
        shader.setFloat("causticScaleFactor", CAUSTIC_SCALE);
        CausticsManager::setTextureSlot(2);
        shader.setInt("clippingMode", clippingMode);
        for (Model &model : models)
        {
            shader.setMat4("model", model.getModelMatrix());
            model.Draw();
        }
    }

private:
    //===============================//
    //          VARIABLES            //
    //===============================//
    Shader shader;
};
