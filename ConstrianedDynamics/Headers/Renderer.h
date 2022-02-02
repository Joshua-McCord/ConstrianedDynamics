#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <array>
#include <set>
#include <algorithm>  // std::copy
#include <math.h>
#include <map>
#include <vector>
#include <string>
#include "Shader.h"

class Renderer
{
public:
	// Common Variables
	//glm::mat4 particleTransformMatrix = glm::mat4(1.0f);
	std::vector<glm::mat4> particleTransformMatricies;
	std::vector<double> particlePositions;

	// Constructor/Destructor
	Renderer(unsigned int n_particles, unsigned int n_constraints, std::vector<double> positions);
	~Renderer();

	// Draw the elements every frame
	void DrawParticles();

	// Update Particle Position
	void UpdateParticleTranslationMatrix(std::vector<double> dp);

private:

	// Common Variables
	glm::mat4 proj = glm::ortho(-2.0f, 2.0f, -2.0f, 2.0f, -1.0f, 1.0f);
	unsigned int n_particles;
	unsigned int n_constraints;

	// VAOS
	std::vector<int> particleVAOs;

	// Render Data Initialization
	void initRenderData(std::vector<double> positions);
	void initParticleRenderer(std::vector<double> positions);

	// Shader Management
	Shader shader;
	unsigned int shaderProgram;
	static std::map<std::string, Shader>    shaderManager;

	static Shader loadShaderFromFile(const char* vShaderFile, const char* fShaderFile, const char* gShaderFile = nullptr);
	Shader LoadShader(const char* vShaderFile, const char* fShaderFile, const char* gShaderFile, std::string name);

	glm::vec3 SetFillFromHUE(float hue);
};
