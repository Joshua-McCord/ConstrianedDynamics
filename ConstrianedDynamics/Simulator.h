#pragma once

#include "Renderer.h"
#include <vector>
#include <map>
#include <iostream>
#include <format>
#include <string>
#include <string_view>
#include <fstream>

#include <format>

#include <GLFW/glfw3.h>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include "exprtk.hpp"

enum FunctionState {
	FUNCTION_ONE,
	FUNCTION_TWO,
	FUNCTION_THREE,
	FUNCTION_FOUR,
	FUNCTION_FIVE,
	FUNCTION_SIX,
	FUNCTION_SEVEN,
	FUNCTION_EIGHT,
	FUNCTION_NINE,
	FUNCTION_ZERO,
};

enum SolverState {
	EULER_SOLVER,
	MIDPOINT_SOLVER,
	RUNGE_KUTTA_FOUR_SOLVER,
};



class Simulator {
public:
	FunctionState	CurrentFunctionState;
	FunctionState	PreviousFunctionState;
	SolverState		CurrentSolverState;
	bool			Keys[1024];
	int				Width, Height;


	Simulator(unsigned int width, unsigned int height);
	~Simulator();

	// init sim
	// --------
	void initSystemData();
	void initRenderer();

	// updating and rendering
	// ----------------------
	void ProcessInput(float dt);
	void ProcessMouseInput(double x, double y);
	void Update(double dt);
	void Render();

	// utilities
	// ---------
	glm::vec3 Function(glm::vec3 pos);
	std::vector<double> Euler_step(double h);
	std::vector<double> Midpoint_Step(double h);
	std::vector<double> Runge_Kutta_Four_Step(double h);


private:
	int n_particles = 1;
	int n_constraints = 1;

	std::vector<double> position_derivatives;
	std::vector<double> velocity_derivatives;

	void GetInitialConditions();
	void GetConstraints();
	void CalculateCDot();
	void CalculateJ();
	void CalculateJDot();
	void EvalDerivative();

	void PrintMatrix(std::vector<std::vector<double>> mat, std::string name);
	void PrintVector(std::vector<double> vec, std::string name);

	std::vector<std::vector<double>> Transpose(std::vector<std::vector<double>> mat);
	std::vector<std::vector<double>> MatMult(std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);
	std::vector<double> MatVecMult(std::vector<std::vector<double>> A, std::vector<double> B);
	std::vector<double> VecSub(std::vector<double> A, std::vector<double> B);
	std::vector<double> VecAdd(std::vector<double> A, std::vector<double> B);
	std::vector<std::vector<double>> MatScalarMult(std::vector<std::vector<double>> mat, double scalar);
	std::vector<double> LinearSolve(std::vector<std::vector<double>> a, std::vector<double> b);

	exprtk::parser<double>		parser_t;
	exprtk::symbol_table<double> symbol_table_t;

	std::vector<exprtk::expression<double>> C;
	std::vector<exprtk::expression<double>> Cdot;
	std::vector<std::vector<double>> J;
	std::vector<std::vector<double>> Jdot;

	std::vector<std::vector<double>> massMatrix;
	std::vector<std::pair<std::string, double>> positions;
	std::vector<std::pair<std::string, double>> velocities;
	std::vector<std::pair<std::string, double>> forces;

};