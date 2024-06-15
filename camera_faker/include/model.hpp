// Modified from: https://learnopengl.com/Model-Loading/Model
#pragma once
#ifndef MODEL_H
#define MODEL_H

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <mesh.hpp>
#include <shader.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

using glm::vec3, glm::vec4, glm::mat4;
using std::string, std::vector, std::cout, std::endl;

enum Units
{
    INCH,
    FEET,
    MILIMETER,
    CENTIMETER,
    METER
};
std::unordered_map<Units, double> unitsToDouble = {
    {Units::INCH, 0.0254},
    {Units::FEET, 0.3048},
    {Units::MILIMETER, 0.001},
    {Units::CENTIMETER, 0.01},
    {Units::METER, 1.0}};

std::unordered_map<string, Units> stringToUnits = {
    {"inch", INCH},
    {"in", INCH},
    {"feet", FEET},
    {"ft", FEET},
    {"millimeter", MILIMETER},
    {"mm", MILIMETER},
    {"centimeter", CENTIMETER},
    {"cm", CENTIMETER},
    {"meter", METER},
    {"m", METER}};

class Model
{
public:
    Model() {}
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    Model(string const &path,
          double units,
          vec3 position = vec3(0, 0, 0),
          vec3 offsetPos = vec3(0, 0, 0),
          vec3 rpy = vec3(0, 0, 0),
          vec3 offsetRPY = vec3(0, 0, 0),
          vec3 color = vec3(-1, -1, -1))
    {
        // Position and orientation
        this->units = units;
        this->position = position;
        this->color = color;
        this->eulerRPY = rpy;
        updateModelMatrix();
        updateOffsetMatrix(offsetPos, offsetRPY);

        // LOAD MODEL
        //---------------------------------------------------------
        // Read file via ASSIMP
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
        // Check for errors
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
        {
            cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
            return;
        }

        // Process ASSIMP's root node recursively
        processNode(scene->mRootNode, scene, mat4(1.0f));
    }

    // Draws the model by drawing all its meshes
    void Draw()
    {
        for (int i = 0; i < static_cast<int>(meshes.size()); i++)
            meshes[i].Draw();
    }

    mat4 getModelMatrix()
    {
        return modelMatrix;
    }

    void setPosition(float x, float y, float z)
    {
        position = vec3(x, y, z);
        updateModelMatrix();
    }
    void setRPY(float roll, float pitch, float yaw)
    {
        eulerRPY = vec3(roll, pitch, yaw);
        updateModelMatrix();
    }

private:
    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene, mat4 parentTransform)
    {
        mat4 nodeTransform = parentTransform * (ai2mat4(node->mTransformation));
        // process each mesh located at the current node
        for (unsigned int i = 0; i < node->mNumMeshes; i++)
        {
            // the node object only contains indices to index the actual objects in the scene.
            // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
            aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            meshes.push_back(processMesh(mesh, scene, nodeTransform));
        }
        // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
        for (unsigned int i = 0; i < node->mNumChildren; i++)
        {
            processNode(node->mChildren[i], scene, mat4(1.0));
        }
    }

    Mesh processMesh(aiMesh *mesh, const aiScene *scene, mat4 transform)
    {
        // data to fill
        vector<Vertex> vertices;
        vector<unsigned int> indices;

        // walk through each of the mesh's vertices
        for (unsigned int i = 0; i < mesh->mNumVertices; i++)
        {
            Vertex vertex;
            vec4 vertexPos = transform * vec4(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z, 1.0f);
            // Positions
            vertex.Position = vec3(offsetMatrix * (vertexPos * (float)units));
            // Normals
            if (mesh->HasNormals())
                vertex.Normal = glm::normalize(glm::mat3(offsetMatrix * transform) * vec3(mesh->mNormals[i].x,
                                                                                          mesh->mNormals[i].y,
                                                                                          mesh->mNormals[i].z));
            // Colors
            aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
            aiColor4D diffuse;
            if (color.r < 0 && AI_SUCCESS == aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
                vertex.Color = vec4(diffuse.r,
                                    diffuse.g,
                                    diffuse.b,
                                    1);
            else if (color.r >= 0)
                vertex.Color = vec4(color, 1.0);
            else
                vertex.Color = vec4(0.2, 0.2, 0.2, 1.0);

            vertices.push_back(vertex);
        }
        // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
        for (unsigned int i = 0; i < mesh->mNumFaces; i++)
        {
            aiFace face = mesh->mFaces[i];
            // retrieve all indices of the face and store them in the indices vector
            for (unsigned int j = 0; j < face.mNumIndices; j++)
                indices.push_back(face.mIndices[j]);
        }

        // return a mesh object created from the extracted mesh data
        return Mesh(vertices, indices);
    }

    // I know this is cursed lol
    mat4 ai2mat4(const aiMatrix4x4 &from)
    {
        mat4 to;
        to[0][0] = from.a1;
        to[0][1] = from.b1;
        to[0][2] = from.c1;
        to[0][3] = from.d1;
        to[1][0] = from.a2;
        to[1][1] = from.b2;
        to[1][2] = from.c2;
        to[1][3] = from.d2;
        to[2][0] = from.a3;
        to[2][1] = from.b3;
        to[2][2] = from.c3;
        to[2][3] = from.d3;
        to[3][0] = from.a4;
        to[3][1] = from.b4;
        to[3][2] = from.c4;
        to[3][3] = from.d4;
        return to;
    }

    void updateModelMatrix()
    {
        modelMatrix = mat4(1.0f);
        // Apply translation
        modelMatrix = glm::translate(modelMatrix, position);
        // Apply Euler angle rotations
        modelMatrix = glm::rotate(modelMatrix, glm::radians(eulerRPY.x), vec3(1, 0, 0)); // Roll
        modelMatrix = glm::rotate(modelMatrix, glm::radians(eulerRPY.y), vec3(0, 1, 0)); // Pitch
        modelMatrix = glm::rotate(modelMatrix, glm::radians(eulerRPY.z), vec3(0, 0, 1)); // Yaw
    }

    void updateOffsetMatrix(vec3 offsetPos, vec3 offsetRPY)
    {
        offsetMatrix = mat4(1.0f);
        // Apply translation
        offsetMatrix = glm::translate(offsetMatrix, offsetPos);
        // Apply Euler angle rotations
        offsetMatrix = glm::rotate(offsetMatrix, glm::radians(offsetRPY.x), vec3(1, 0, 0)); // Roll
        offsetMatrix = glm::rotate(offsetMatrix, glm::radians(offsetRPY.y), vec3(0, 1, 0)); // Pitch
        offsetMatrix = glm::rotate(offsetMatrix, glm::radians(offsetRPY.z), vec3(0, 0, 1)); // Yaw
    }

    // model data
    vec3 color;
    double units;
    vec3 eulerRPY;
    vec3 position;
    vector<Mesh> meshes;
    mat4 modelMatrix;
    mat4 offsetMatrix;
};
#endif
