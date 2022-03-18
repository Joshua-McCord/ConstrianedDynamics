#pragma once
#define _CRT_SECURE_NO_DEPRECATE
#include "Renderer.h"
#include "MathLib.h"
#include "Parser.h"
#include "Logger.h"
#include <vector>
#include <map>
#include <iostream>
#include <format>
#include <string>
#include <string_view>
#include <fstream>
#include <sstream>

#include <format>

#include <GLFW/glfw3.h>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include <exprtk/exprtk.hpp>
#include <Python.h>

enum FunctionState {
	FUNCTION_ONE
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
	void initSystem();
	void initRenderer();

	// updating and rendering
	// ----------------------
	void Update(double dt);
	void Render();

	// utilities
	// ---------
	std::vector<double> Euler_step(double h);
	std::vector<double> Runge_Kutta_Four_Step(double h);



private:
	// Global Simulator State Data
	// ---------------------------
	int n_particles = 1;
	int n_constraints = 1;

	void GetInitialConditions();
	void CalculateJ();
	void CalculateJDot();
	void EvalDerivative();

	std::vector<double> position_derivatives;
	std::vector<double> velocity_derivatives;

	std::vector<exprtk::expression<double>> C;
	std::vector<exprtk::expression<double>> Cdot;
	std::vector<std::vector<double>> J;
	std::vector<std::vector<exprtk::expression<double>>> JFile;
	std::vector<std::vector<double>> Jdot;

	std::vector<std::vector<double>> massMatrix;
	std::vector<std::pair<std::string, double>> positions;
	std::vector<std::pair<std::string, double>> velocities;
	std::vector<std::pair<std::string, double>> forces;

	// Utilities
	// ---------
	exprtk::parser<double>		parser_t;
	exprtk::symbol_table<double> symbol_table_t;
	
};