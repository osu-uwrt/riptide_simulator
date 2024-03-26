#pragma once
#include <glm/glm.hpp>

// NOT USED, IMPLEMENT MAYBE
// NOT USED, IMPLEMENT MAYBE
// NOT USED, IMPLEMENT MAYBE
// NOT USED, IMPLEMENT MAYBE

enum lightType
{
    DIR_LIGHT,
    POINT_LIGHT,
    SPOT_LIGHT
};

class Light
{
public:
    Light(lightType type,
          glm::vec3 ambient = glm::vec3(0.0f),
          glm::vec3 diffuse = glm::vec3(1.0f),
          glm::vec3 specular = glm::vec3(1.0f),
          glm::vec3 position = glm::vec3(0.0f),
          glm::vec3 direction = glm::vec3(0.0f, -1.0f, 0.0f),
          float cutOff = glm::cos(glm::radians(12.5f)),
          float outerCutOff = glm::cos(glm::radians(17.5f)),
          float constant = 1.0f,
          float linear = 0.09f,
          float quadratic = 0.032f)
        : type(type),
          ambient(ambient), diffuse(diffuse), specular(specular),
          position(position), direction(direction), cutOff(cutOff), outerCutOff(outerCutOff),
          constant(constant), linear(linear), quadratic(quadratic)
    {
    }

private:
    // Used for spot and directional lights
    glm::vec3 direction;

    // Used for spot lights only
    float cutOff;
    float outerCutOff;

    // Used for point and spot lights
    glm::vec3 position;
    float constant;
    float linear;
    float quadratic;

    // Used for all light types
    lightType type;
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;
};
