#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "shader.hpp"
#include "camera.hpp"
#include "object.hpp"
#include <cmath>
#include "model.hpp"

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
     * @param model The 3D model that will be drawn
     * @param camera the camera that is being used to look at the model
     */
    void render(Model model, Camera camera)
    {
        // Activate shader
        shader.use();
        glEnable(GL_DEPTH_TEST);
        // Get camera matrixs
        shader.setMat4("model", model.getModelMatrix());
        shader.setMat4("view", camera.getViewMatrix());
        shader.setMat4("projection", camera.getProjectionMatrix());

        // Set uniform variables that are constant for all objects
        shader.setVec3("cameraPos", camera.getPosition());
        shader.setFloat("fogStrength", FOG_STRENGTH);
        shader.setVec4("fogColor", glm::vec4(FOG_COLOR, 1.0));

        model.Draw();
    }

private:
    //===============================//
    //          VARIABLES            //
    //===============================//
    Shader shader;
};
