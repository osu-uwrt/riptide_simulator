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
    void render(Model model, Camera camera)
    {
        // Activate shader
        shader.use();
        glEnable(GL_DEPTH_TEST);
        // Get camera matrixs
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)IMG_WIDTH / (float)IMG_HEIGHT, 0.1f, 100.0f);
        shader.setMat4("model", model.getModelMatrix());
        shader.setMat4("view", camera.getViewMatrix());
        shader.setMat4("projection", projection);

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
