#include "../Headers/Simulator.h"

#include <algorithm>


Renderer* renderer;
Parser* parser;

Simulator::Simulator(unsigned int width, unsigned int height) : CurrentFunctionState(FUNCTION_ONE), CurrentSolverState(EULER_SOLVER), Keys(), Width(width), Height(height) {
	PreviousFunctionState = CurrentFunctionState;
}

// Simulator
// ---------
void Simulator::initSystem() {
	// Generate Data
	Py_Initialize();
	FILE* fd = fopen("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/GenConstraintData.py", "r");
	// last parameter == 1 means to close the file before returning.
	PyRun_SimpleFileEx(fd, "C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/GenConstraintData.py", 1);
	Py_Finalize();

	// Load Data
	this->GetInitialConditions();
	parser->ParseFile("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/constraints.txt", &(this->C), symbol_table_t);
	parser->ParseFile("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/constraints_dot.txt", &(this->Cdot), symbol_table_t);
	this->CalculateJ();
	this->CalculateJDot();

	this->initRenderer();

}
void Simulator::GetInitialConditions() {
	// Read in Positions
	parser->ParseFile("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/positions.txt", &(this->positions), symbol_table_t, ' ');

	// Read in Velocities
	parser->ParseFile("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/velocities.txt", &(this->velocities), symbol_table_t, 'v');

	// Read in Forces
	parser->ParseFile("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/forces.txt", &(this->forces), symbol_table_t, 'f');

	// Read in Masses
	parser->ParseFile("C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/masses.txt", &(this->massMatrix), symbol_table_t);
}
void Simulator::CalculateJ() {
	// We're evaluating every loop...don't want to end up with impossibly long vector
	this->J.clear();
	for (exprtk::expression<double> ex : this->C) {
		// All positions are in a map
		// Since these are vector values functions, we can take the derivative with respect to every value in positions, and sum them up
		std::vector<double> C_der;
		for (auto it = this->positions.begin(); it != this->positions.end(); ++it) {
			auto pos = it->first;
			double derivative = exprtk::derivative(ex, pos);
			C_der.push_back(derivative);
		}
		this->J.push_back(C_der);
	}
}
void Simulator::CalculateJDot() {
	// We're evaluating every loop...don't want to end up with impossibly long vector
	this->Jdot.clear();
	for (exprtk::expression<double> ex : this->Cdot) {
		// All positions are in a map
		// Since these are vector values functions, we can take the derivative with respect to every value in positions, and sum them up
		std::vector<double> C_dot_der;
		for (int i = 0; i < this->positions.size(); i++) {
			auto pos = this->positions[i].first;

			double derivative = exprtk::derivative(ex, pos);
			C_dot_der.push_back(derivative);
		}
		this->Jdot.push_back(C_dot_der);
	}
}

// Renderer
// --------
void Simulator::initRenderer() {
	std::vector<double> pos;
	for (auto i : this->positions) {
		pos.push_back(i.second);
	}
	unsigned int n_particles = this->positions.size() / 2;
	unsigned int n_constraints = this->C.size();
	renderer = new Renderer(n_particles, n_constraints, pos);
	parser = new Parser();
}

void Simulator::Update(double dt) {
	double step_size = 0.005f;
	renderer->UpdateParticleTranslationMatrix(this->Runge_Kutta_Four_Step(step_size));
}
void Simulator::Render() {
	renderer->DrawParticles();
}



// Euler Solver (this maybe able to be void since using global particle system
std::vector<double> Simulator::Euler_step(double h) {
	this->EvalDerivative();

	auto pos = this->positions;
	auto vel = this->velocities;

	auto dp = this->position_derivatives;
	auto dv = this->velocity_derivatives;

	for (int i = 0; i < dp.size(); i++) {
		dp[i] *= h;
		dv[i] *= h;
	}

	int cnt = 0;
	for (auto it = this->positions.begin(); it != this->positions.end(); ++it) {
		it->second += dp[cnt];
		cnt++;
	}

	cnt = 0;
	for (auto it = this->velocities.begin(); it != this->velocities.end(); ++it) {
		it->second += dv[cnt];
		cnt++;
	}
	return dp;
}
// RK4 Solver
std::vector<double> Simulator::Runge_Kutta_Four_Step(double h) {
	//return glm::vec3(0.0f);
	auto original_positions = this->positions;
	auto original_velocities = this->velocities;

	// -------------- RK1 --------------
	// ---------------------------------
	this->EvalDerivative();
	auto dp = this->position_derivatives;
	auto dv = this->velocity_derivatives;

	int cnt = 0;
	for (auto it = this->positions.begin(); it != this->positions.end(); ++it) {
		it->second = original_positions[cnt].second + ((dp[cnt] * h) / 2);
		cnt++;
	}
	cnt = 0;
	for (auto it = this->velocities.begin(); it != this->velocities.end(); ++it) {
		it->second = original_velocities[cnt].second + ((dv[cnt] * h) / 2);
		cnt++;
	}

	// -------------- RK2 --------------
	// ---------------------------------
	this->EvalDerivative();
	dp = this->position_derivatives;
	dv = this->velocity_derivatives;

	cnt = 0;
	for (auto it = this->positions.begin(); it != this->positions.end(); ++it) {
		it->second = original_positions[cnt].second + ((dp[cnt] * h) / 2);
		cnt++;
	}
	cnt = 0;
	for (auto it = this->velocities.begin(); it != this->velocities.end(); ++it) {
		it->second = original_velocities[cnt].second + ((dv[cnt] * h) / 2);
		cnt++;
	}

	// -------------- RK3 --------------
	// ---------------------------------
	this->EvalDerivative();
	dp = this->position_derivatives;
	dv = this->velocity_derivatives;

	cnt = 0;
	for (auto it = this->positions.begin(); it != this->positions.end(); ++it) {
		it->second = original_positions[cnt].second + ((dp[cnt] * h) / 2);
		cnt++;
	}
	cnt = 0;
	for (auto it = this->velocities.begin(); it != this->velocities.end(); ++it) {
		it->second = original_velocities[cnt].second + ((dv[cnt] * h) / 2);
		cnt++;
	}

	// -------------- RK4 --------------
	// ---------------------------------
	this->EvalDerivative();
	dp = this->position_derivatives;
	dv = this->velocity_derivatives;

	cnt = 0;
	for (auto it = this->positions.begin(); it != this->positions.end(); ++it) {
		it->second = original_positions[cnt].second + ((dp[cnt] * h));
		cnt++;
	}
	cnt = 0;
	for (auto it = this->velocities.begin(); it != this->velocities.end(); ++it) {
		it->second = original_velocities[cnt].second + ((dv[cnt] * h));
		cnt++;
	}

	for (int i = 0; i < dp.size(); i++) {
		dp[i] *= h;
	}
	return dp;
}

void Simulator::EvalDerivative() {
	auto C = this->C;

	auto Cdot = this->Cdot;

	this->CalculateJ();
	auto J = this->J;

	this->CalculateJDot();
	auto Jdot = this->Jdot;

	std::vector<double> positions_derivative;
	std::vector<double> velocities_derivative;

	// Feedback Terms
	double ks = 0.1;
	double kd = 0.05;

	//J^T
	std::vector<std::vector<double>> j_transpose = Transpose(J);
	//W
	std::vector<std::vector<double>> W(this->massMatrix.size());
	for (auto it = W.begin(); it != W.end(); ++it) {
		it->resize(this->massMatrix[0].size());
	}
	for (int i = 0; i < W.size(); i++) {
		for (int j = 0; j < W[0].size(); j++) {
			double val = massMatrix[i][j];
			if (val != 0.0f) {
				W[i][j] = 1.0f / val;
			}
			else {
				W[i][j] = 0.0f;
			}
		}
	}

	// Linear System
	//this->PrintMatrix(this->MatMult(this->MatMult(J, W), j_transpose), "J*W*Jt");
	std::vector<std::vector<double>> linear_system = MatMult(MatMult(J, W), j_transpose);

	// -Jdot
	std::vector<std::vector<double>> neg_j_dot(Jdot.size());
	for (auto it = neg_j_dot.begin(); it != neg_j_dot.end(); it++) {
		it->resize(Jdot[0].size());
	}
	for (int i = 0; i < Jdot.size(); i++) {
		for (int j = 0; j < Jdot[i].size(); j++) {
			neg_j_dot[i][j] = -1 * Jdot[i][j];
		}
	}

	// q_dot
	std::vector<double> q_dot;
	for (auto it = this->velocities.begin(); it != this->velocities.end(); ++it) {
		q_dot.push_back(it->second);
	}
	std::vector<double> Q;
	for (auto it = this->forces.begin(); it != this->forces.end(); ++it) {
		Q.push_back(it->second);
	}

	std::vector<double> a = MatVecMult(neg_j_dot, q_dot);
	std::vector<double> b = MatVecMult(W, Q);
	std::vector<double> c = MatVecMult(J, b);
	std::vector<double> feedback1;
	for (int i = 0; i < C.size(); i++) {
		auto val = C[i].value();
		feedback1.push_back(val *= ks);
	}
	std::vector<double> feedback2;
	for (int i = 0; i < C.size(); i++) {
		auto val = Cdot[i].value();
		feedback2.push_back(val *= kd);
	}

	auto feedback = VecSub(feedback1, feedback2);
	std::vector<double> linear_solution = VecSub(a, c);
	//this->PrintVector(linear_solution, "Linear Solution");

	std::vector<double> lambda = LinearSolve(linear_system, linear_solution);

	auto const_force = MatVecMult(j_transpose, lambda);
	auto total_force = VecAdd(Q, const_force);

	this->position_derivatives = q_dot;	// q_dot = velocities
	this->velocity_derivatives = MatVecMult(W, total_force);
}

void Simulator::PrintMatrix(std::vector<std::vector<double>> mat, std::string name) {
	std::cout << name << " = " << std::endl;
	std::cout << "[";
	for (int i = 0; i < mat.size(); ++i) {
		std::cout << "[";
		for (int j = 0; j < mat[0].size(); ++j) {
			std::cout << mat[i][j];
			if (j != mat[0].size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "]";
		if (i != mat.size() - 1) {
			std::cout << ",\n";
		}
	}
	std::cout << "]" << std::endl;
}
void Simulator::PrintVector(std::vector<double> vec, std::string name) {
	std::cout << name << " = " << std::endl;
	std::cout << "[";
	for (int i = 0; i < vec.size(); ++i) {
		std::cout << vec[i];
		if (i != vec.size() - 1) {
			std::cout << ", ";
		}
	}
	std::cout << "]" << std::endl;
}