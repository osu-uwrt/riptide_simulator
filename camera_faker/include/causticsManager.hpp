#pragma once
#include <iostream>
#include <fstream>
#include <settings.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <string>
#include <filesystem>
#include <texture.hpp>

namespace fs = std::filesystem;
using std::string;

namespace CausticsManager
{
    // Static variables
    std::vector<Texture> causticsImgs;
    bool texturesLoaded = false;

    // Function to load caustic textures
    void loadCaustics(const string &causticPath)
    {
        if (texturesLoaded)
            return;

        // Loop through each file in the caustics folder
        int i = 1;
        for (const auto &entry : fs::directory_iterator(causticPath))
        {
            if (entry.path().extension() != ".jpg" && entry.path().extension() != ".png")
            {
                std::cout << "WHOA there's a non jpg or png caustics file dawg: " << entry.path().filename() << std::endl;
                continue;
            }
            // Load images as texture and add it to the "list"
            string fileName = "Caustics_Texture (" + std::to_string(i) + ").jpg";
            if (!fs::exists(fs::path(causticPath) / fileName))
                continue;
            string texturePath = fs::path(causticPath) / fileName;
            causticsImgs.push_back(Texture(texturePath, false));
            i++;
        }
        texturesLoaded = true;
        std::cout << "Done loading caustic textures" << std::endl;
    }

    // Function to get the current caustic texture ID based on time
    void setTextureSlot(int textureSlot)
    {
        if (causticsImgs.empty())
            throw std::runtime_error("Caustic textures not loaded");
        unsigned int index = ((int)(CAUSTIC_SPEED * glfwGetTime())) % causticsImgs.size();
        causticsImgs[index].use(textureSlot);
    }
}