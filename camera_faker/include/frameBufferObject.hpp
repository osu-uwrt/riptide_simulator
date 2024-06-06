#pragma once
#include <glad/glad.h>
#include <chrono>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <iostream>
#include "settings.h"
using std::vector;

class FBO
{
public:
    FBO() {}
    FBO(bool screen_)
    {
        // Screen is a bool that holds whether the screen is the default screen buffer or not
        screen = screen_;
        if (screen)
            // This is a screen buffer, make the framebuffer ID = 0 which is the default screen
            framebuffer = 0;
        else
        {
            // Frame buffer setup
            glGenFramebuffers(1, &framebuffer);
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

            // generate color texture
            glGenTextures(1, &colorTextureBuffer);
            glBindTexture(GL_TEXTURE_2D, colorTextureBuffer);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, IMG_WIDTH, IMG_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);

            // Set up render buffer object for depth stencil
            glGenRenderbuffers(1, &rbo);
            glBindRenderbuffer(GL_RENDERBUFFER, rbo);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, IMG_WIDTH, IMG_HEIGHT);
            glBindRenderbuffer(GL_RENDERBUFFER, 0);

            // Set up depth texture for holding camera depth data (Can be values not 0-1 unlike depth stencil for depth testing)
            glGenTextures(1, &depthTextureBuffer);
            glBindTexture(GL_TEXTURE_2D, depthTextureBuffer);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, IMG_WIDTH, IMG_HEIGHT, 0, GL_RED, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // Attach buffers to frame buffer object
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTextureBuffer, 0);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, depthTextureBuffer, 0);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

            // Check if all features of the framebuffer are filled in properly
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }
    }
    void use()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        glDrawBuffers(2, drawBuffers);
    }
    void useTexture(int textureSlot, bool useDepth = false)
    {
        // Make the desired texture slot active so that the texture gets bound to the correct place
        glActiveTexture(GL_TEXTURE0 + textureSlot);
        if (!useDepth)
            // Sets texture to the color buffer
            glBindTexture(GL_TEXTURE_2D, colorTextureBuffer);
        else
            // Sets texture to the depth texture
            glBindTexture(GL_TEXTURE_2D, depthTextureBuffer);
    }
    void clear()
    {
        use();
        // Clear screen to skyish color
        glClearColor(0.568f, 0.878f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    int textureID()
    {
        return colorTextureBuffer;
    }
    void copyColorData(std::vector<uint8_t> &imgData)
    {
        use();
        // Copy color information
        int numChannels = 3; // RGB
        size_t data_size = IMG_HEIGHT * IMG_WIDTH * numChannels;
        imgData.resize(data_size);
        glBindTexture(GL_TEXTURE_2D, colorTextureBuffer);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, imgData.data());

        // Flip the image vertically
        for (int i = 0; i < IMG_HEIGHT / 2; ++i)
        {
            std::swap_ranges(imgData.begin() + i * IMG_WIDTH * numChannels,
                             imgData.begin() + (i + 1) * IMG_WIDTH * numChannels,
                             imgData.begin() + (IMG_HEIGHT - i - 1) * IMG_WIDTH * numChannels);
        }
        glBindTexture(GL_TEXTURE_2D, colorTextureBuffer);
    }

    void copyDepthData(std::vector<uint8_t> &depthData)
    {
        use();
        // Copy depth information
        size_t data_size = IMG_HEIGHT * IMG_WIDTH * sizeof(float);
        depthData.resize(data_size);
        glBindTexture(GL_TEXTURE_2D, depthTextureBuffer);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, depthData.data());

        // Flip the image vertically
        for (int i = 0; i < IMG_HEIGHT / 2; ++i)
        {
            std::swap_ranges(depthData.begin() + i * IMG_WIDTH * sizeof(float),
                             depthData.begin() + (i + 1) * IMG_WIDTH * sizeof(float),
                             depthData.begin() + (IMG_HEIGHT - i - 1) * IMG_WIDTH * sizeof(float));
        }
        glBindTexture(GL_TEXTURE_2D, depthTextureBuffer);
    }

private:
    //===============================//
    //          VARIABLES            //
    //===============================//
    unsigned int rbo,
        framebuffer,
        colorTextureBuffer,
        depthTextureBuffer;
    bool screen;
};
