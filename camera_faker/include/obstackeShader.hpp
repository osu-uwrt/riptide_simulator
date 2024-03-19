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
namespace fs = std::filesystem;
using std::string;

class ObjectShader
{
public:
    ObjectShader() {}
    ObjectShader(string vsPath, string fsPath, string causticsPath)
    {
        shader = Shader(vsPath, fsPath);
        loadCaustics(causticsPath);
    }
    void render(std::vector<Object> objects, Camera camera)
    {
        // Activate shader
        shader.use();
        glEnable(GL_DEPTH_TEST);
        //  Get camera matrixs
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)IMG_WIDTH / (float)IMG_HEIGHT, 0.1f, 100.0f);
        shader.setMat4("projection", projection);
        shader.setMat4("view", camera.GetViewMatrix());

        // Set uniform variables that are constant for all objects
        shader.setVec3("cameraPos", camera.Position);
        shader.setFloat("fogStrength", FOG_STRENGTH);
        shader.setFloat("causticStrength", CAUSTIC_STRENGTH);
        shader.setVec4("fogColor", glm::vec4(FOG_COLOR, 1.0));
        shader.setFloat("fogFactor", FOG_STRENGTH);
        shader.setInt("ourTexture", 1);
        shader.setInt("causticTexture", 2);
        // Figure out what caustic image from the animation to use then use it
        unsigned int index = ((int)(CAUSTIC_SPEED * glfwGetTime())) % causticsImgs.size();
        causticsImgs[index].use(2);
        // Set uniform stuff for directional light

        // Loop through each object to render
        for (Object object : objects)
        {
            object.use();
            shader.setMat4("model", object.getModelMatrix());
            // Draw two triangles on the screen
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }
    }

private:
    void loadCaustics(string causticPath)
    {
        // Vector to hold directory entries
        std::vector<fs::directory_entry> entries;
        // Populate the vector with directory entries
        int i = 1;
        for (const auto &entry : fs::directory_iterator(causticPath))
        {
            // Check if the file in the directory is a valid image first
            if (entry.path().extension() != ".jpg" && entry.path().extension() != ".png")
            {
                std::cout << "WHOA there's a non jpg or png caustics file dawg: " << entry.path().filename() << std::endl;
                continue;
            }
            // Kinda hard coding the file names here, but it works
            string fileName = "Caustics_Texture (" + std::to_string(i) + ").jpg";
            string texturePath = fs::path(causticPath) / fileName;
            // Make a texture for the file and add it to the list
            causticsImgs.push_back(Texture(texturePath));
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            i++;
        }
        std::cout << "done loading textures" << std::endl;
    }

    std::vector<Texture> causticsImgs;
    Shader shader;
};
