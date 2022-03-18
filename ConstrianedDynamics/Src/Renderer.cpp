#include "../Headers/Renderer.h"

#include <iostream>
#include <sstream>
#include <fstream>

std::map<std::string, Shader>       Renderer::shaderManager;

Renderer::Renderer(unsigned int n_particles, unsigned int n_constraints, std::vector<double> positions) {
	this->n_particles = n_particles;
	this->n_constraints = n_constraints;
	this->initRenderData(positions);
}

Renderer::~Renderer() {
}

// Load and Store Shaders and Initialize all Renderers
// ---------------------------------------------------
void Renderer::initRenderData(std::vector<double> positions) {
	this->LoadShader("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Shaders/Particle.vert",
		"C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Shaders/Particle.frag",
		nullptr,
		"particleShader");
	this->initParticleRenderer(positions);
}

// Initialize, Draw, and Update Particle
// -----------------------------------------
// TODO: This should be done in the frag shader; this makes it not smooth and wastes memory
void Renderer::initParticleRenderer(std::vector<double> positions) {
	for (int i = 0; i < positions.size() - 1; i += 2) {
		glm::mat4 transform(1);
		glm::vec3 trns = glm::vec3(positions[i], positions[i + 1], 0.0f);
		transform = glm::translate(transform, trns);
		this->particleTransformMatricies.push_back(transform);
	}

	std::cout << this->particleVAOs.size() << std::endl;
	for (int ind = 0; ind < this->n_particles; ind++) {
		float vertices[2400];
		auto i = 0;
		auto x = 0.0f,
			y = x,
			z = x,
			r = 0.025f;

		auto numSides = 84;
		auto TWO_PI = 2.0f * 3.1415f;
		auto increment = TWO_PI / numSides;

		for (auto angle = 0.0f; angle <= TWO_PI; angle += increment) {
			vertices[i++] = r * cos(angle) + x;
			vertices[i++] = r * sin(angle) + y;
			vertices[i++] = 0.0f;

			vertices[i++] = 0.0f;
			vertices[i++] = 0.0f;
			vertices[i++] = 0.0f;

			vertices[i++] = r * cos(angle + increment) + x;
			vertices[i++] = r * sin(angle + increment) + y;
			vertices[i++] = 0.0f;
		}
		unsigned int VBO, particleVAO;
		glGenVertexArrays(1, &particleVAO);
		glGenBuffers(1, &VBO);
		glBindVertexArray(particleVAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 800, vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		this->particleVAOs.push_back(particleVAO);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindVertexArray(0);
	}
}
void Renderer::DrawParticles() {
	for (int i = 0; i < particleVAOs.size(); i++) {
		// prepare transformations
		this->shader = shaderManager["particleShader"];
		this->shader.Use();
		this->shader.SetMatrix4("projection", this->proj);
		this->shader.SetMatrix4("model", this->particleTransformMatricies[i]);

		//// draw ball
		glUseProgram(this->shader.ID);
		glBindVertexArray(this->particleVAOs[i]);
		glDrawArrays(GL_TRIANGLES, 0, 2400);
		glBindVertexArray(0);
	}
}
void Renderer::UpdateParticleTranslationMatrix(std::vector<double> dp) {
	int cnt = 0;
	for (int i = 0; i < this->particleTransformMatricies.size(); i++) {
		//this->particleTransformMatricies[i] = glm::mat4(1.0f);
		glm::mat4 mod = this->particleTransformMatricies[i];
		mod = glm::translate(mod, glm::vec3(dp[cnt], dp[cnt + 1], 0));
		this->particleTransformMatricies[i] = mod;
		cnt++;
		cnt++;
	}
}

// Various Shader Utilities
// ------------------------
Shader Renderer::loadShaderFromFile(const char* vShaderFile, const char* fShaderFile, const char* gShaderFile)
{
	// 1. retrieve the vertex/fragment source code from filePath
	std::string vertexCode;
	std::string fragmentCode;
	std::string geometryCode;
	try
	{
		// open files
		std::ifstream vertexShaderFile(vShaderFile);
		std::ifstream fragmentShaderFile(fShaderFile);
		std::stringstream vShaderStream, fShaderStream;
		// read file's buffer contents into streams
		vShaderStream << vertexShaderFile.rdbuf();
		fShaderStream << fragmentShaderFile.rdbuf();
		// close file handlers
		vertexShaderFile.close();
		fragmentShaderFile.close();
		// convert stream into string
		vertexCode = vShaderStream.str();
		fragmentCode = fShaderStream.str();
		// if geometry shader path is present, also load a geometry shader
		if (gShaderFile != nullptr)
		{
			std::ifstream geometryShaderFile(gShaderFile);
			std::stringstream gShaderStream;
			gShaderStream << geometryShaderFile.rdbuf();
			geometryShaderFile.close();
			geometryCode = gShaderStream.str();
		}
	}
	catch (std::exception e)
	{
		std::cout << "ERROR::SHADER: Failed to read shader files" << std::endl;
	}
	const char* vShaderCode = vertexCode.c_str();
	const char* fShaderCode = fragmentCode.c_str();
	const char* gShaderCode = geometryCode.c_str();
	// 2. now create shader object from source code
	Shader shader;
	shader.Compile(vShaderCode, fShaderCode, gShaderFile != nullptr ? gShaderCode : nullptr);
	return shader;
}
Shader Renderer::LoadShader(const char* vShaderFile, const char* fShaderFile, const char* gShaderFile, std::string name)
{
	shaderManager[name] = loadShaderFromFile(vShaderFile, fShaderFile, gShaderFile);
	return shaderManager[name];
}