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

using std::string, std::vector, std::cout, std::endl;

enum Units
{
    INCH,
    FEET,
    MILIMETER,
    CENTIMETER,
    METER
};
std::map<Units, double> scaleFactor = {
    {Units::INCH, 0.0254},
    {Units::FEET, 0.3048},
    {Units::MILIMETER, 0.001},
    {Units::CENTIMETER, 0.01},
    {Units::METER, 1.0}};

class Model
{
public:
    Model() {}
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    Model(string const &path, Units units, glm::vec3 position = glm::vec3(0, 0, 0), glm::vec3 rpy = glm::vec3(0, 0, 0))
    {
        // Position and orientation
        this->units = units;
        this->position = position;
        eulerRPY = rpy;
        updateModelMatrix();

        // LOAD MODEL
        //---------------------------------------------------------
        // Read file via ASSIMP
        Assimp::Importer importer;
        cout << path << endl;
        const aiScene *scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
        // Check for errors
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
        {
            cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
            return;
        }

        // Process ASSIMP's root node recursively
        processNode(scene->mRootNode, scene, glm::mat4(1.0f));
    }

    // Draws the model by drawing all its meshes
    void Draw()
    {
        for (int i = 0; i < meshes.size(); i++)
            meshes[i].Draw();
    }

    glm::mat4 getModelMatrix()
    {
        return modelMatrix;
    }

    void setPosition(float x, float y, float z)
    {
        position = glm::vec3(x, y, z);
        updateModelMatrix();
    }
    void setRPY(float roll, float pitch, float yaw)
    {
        eulerRPY = glm::vec3(roll, pitch, yaw);
        updateModelMatrix();
    }

private:
    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene, glm::mat4 parentTransform)
    {
        glm::mat4 nodeTransform = parentTransform * (ai2mat4(node->mTransformation));
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
            processNode(node->mChildren[i], scene, glm::mat4(1.0));
        }
    }

    Mesh processMesh(aiMesh *mesh, const aiScene *scene, glm::mat4 transform)
    {
        // data to fill
        vector<Vertex> vertices;
        vector<unsigned int> indices;

        // walk through each of the mesh's vertices
        for (unsigned int i = 0; i < mesh->mNumVertices; i++)
        {
            Vertex vertex;
            glm::vec3 baseLinkOffset(-0.14, 0.03, -0.09);

            glm::vec4 vertexPos = transform * glm::vec4(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z, 1.0f);
            // Positions
            vertex.Position = glm::vec3(scaleFactor[units] * vertexPos.x,
                                        scaleFactor[units] * vertexPos.y,
                                        scaleFactor[units] * vertexPos.z) -
                              baseLinkOffset;
            // Normals
            if (mesh->HasNormals())
                vertex.Normal = glm::normalize(glm::mat3(transform) * glm::vec3(mesh->mNormals[i].x,
                                                                                mesh->mNormals[i].y,
                                                                                mesh->mNormals[i].z));
            // Colors
            aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
            aiColor4D diffuse;
            if (AI_SUCCESS == aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
                vertex.Color = glm::vec4(diffuse.r,
                                         diffuse.g,
                                         diffuse.b,
                                         diffuse.a);
            else
                vertex.Color = glm::vec4(0.2, 0.2, 0.2, 1.0);

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
    glm::mat4 ai2mat4(const aiMatrix4x4 &from)
    {
        glm::mat4 to;
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
        modelMatrix = glm::mat4(1.0f);
        // Apply translation
        modelMatrix = glm::translate(modelMatrix, position);
        // Apply Euler angle rotations
        modelMatrix = glm::rotate(modelMatrix, eulerRPY.x, glm::vec3(1, 0, 0)); // Roll
        modelMatrix = glm::rotate(modelMatrix, eulerRPY.y, glm::vec3(0, 1, 0)); // Pitch
        modelMatrix = glm::rotate(modelMatrix, eulerRPY.z, glm::vec3(0, 0, 1)); // Yaw
    }

    // model data
    glm::vec3 eulerRPY;
    glm::vec3 position;
    vector<Mesh> meshes;
    glm::mat4 modelMatrix;
    Units units;
};
#endif
