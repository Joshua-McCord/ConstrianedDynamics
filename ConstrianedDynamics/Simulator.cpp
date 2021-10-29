#include "Simulator.h"


#include <algorithm>


Renderer* renderer;

Simulator::Simulator(unsigned int width, unsigned int height) : CurrentFunctionState(FUNCTION_ONE), CurrentSolverState(EULER_SOLVER), Keys(), Width(width), Height(height) {
	PreviousFunctionState = CurrentFunctionState;
}

Simulator::~Simulator() {
}

void Simulator::initSystemData() { 
	this->GetInitialConditions();
	this->GetConstraints();
	this->CalculateCDot();
	this->CalculateJ();
	this->CalculateJDot(); 
	this->EvalDerivative();
}

void Simulator::initRenderer() {
	std::vector<double> pos;
	for (auto i : this->positions) {
		pos.push_back(i.second);
	}
	unsigned int n_particles = this->positions.size() / 2;
	unsigned int n_constraints = this->C.size();
	renderer = new Renderer(n_particles, n_constraints, pos);
}


void Simulator::Update(double dt) {
	double step_size = 0.005f;
	renderer->UpdateParticleTranslationMatrix(this->Runge_Kutta_Four_Step(step_size));
}

void Simulator::Render() {
	renderer->DrawParticles();
}


void Simulator::GetInitialConditions() {
	// Read in Positions
	std::ifstream positions_file;
	positions_file.open("positions.txt");
	std::string line;
	if (positions_file.is_open())
	{
		int cnt = 1;
		while (std::getline(positions_file, line))
		{
			std::string delimiter = ",";
			size_t last = 0;
			size_t next = 0;

			while ((next = line.find(delimiter, last)) != std::string::npos) {

				this->positions.push_back(std::pair<std::string, double>(std::format("x{}", cnt), double(std::atof(line.substr(last, next - last).c_str()))));
				last = next + 1;
			}
			this->positions.push_back(std::pair<std::string, double>(std::format("y{}", cnt), double(std::atof(line.substr(last).c_str()))));
			cnt++;
		}
		positions_file.close();
	}

	// Add all positions to symbol table
	for (auto it = this->positions.begin(); it != this->positions.end(); ++it) {
		symbol_table_t.add_variable(it->first, it->second);
	}

	// Read in Velocities
	std::ifstream velocities_file;
	velocities_file.open("velocities.txt");
	if (velocities_file.is_open())
	{
		int cnt = 1;
		while (std::getline(velocities_file, line))
		{
			std::string delimiter = ",";
			size_t last = 0;
			size_t next = 0;

			while ((next = line.find(delimiter, last)) != std::string::npos) {

				this->velocities.push_back(std::pair<std::string, double>(std::format("vx{}", cnt), std::atof(line.substr(last, next - last).c_str())));
				last = next + 1;
			}
			this->velocities.push_back(std::pair<std::string, double>(std::format("vy{}", cnt), std::atof(line.substr(last).c_str())));
			cnt++;
		}
		velocities_file.close();
	}

	// Add all velocities to symbol table
	for (auto it = this->velocities.begin(); it != this->velocities.end(); ++it) {
		symbol_table_t.add_variable(it->first, it->second);
	}


	// Read in Forces
	std::ifstream forces_file;
	forces_file.open("forces.txt");
	if (forces_file.is_open())
	{
		int cnt = 1;
		while (std::getline(forces_file, line))
		{
			std::string delimiter = ",";
			size_t last = 0;
			size_t next = 0;

			while ((next = line.find(delimiter, last)) != std::string::npos) {

				this->forces.push_back(std::pair<std::string, double>(std::format("fx{}", cnt), std::atof(line.substr(last, next - last).c_str())));
				last = next + 1;
			}
			this->forces.push_back(std::pair<std::string, double>(std::format("fy{}", cnt), std::atof(line.substr(last).c_str())));
			cnt++;
		}
		forces_file.close();
	}

	//// Do I need this for forces? 
	//for (std::map<std::string, double>::iterator it = this->velocities.begin(); it != this->velocities.end(); ++it) {
	//	symbol_table_t.add_variable(it->first, it->second);
	//}

	// Read in Masses
	std::vector<double> mass_vector;
	std::ifstream mass_file;
	mass_file.open("masses.txt");
	if (mass_file.is_open())
	{
		int cnt = 1;
		while (std::getline(mass_file, line))
		{
			std::string delimiter = ",";
			size_t last = 0;
			size_t next = 0;

			mass_vector.clear();
			while ((next = line.find(delimiter, last)) != std::string::npos) {

				mass_vector.insert(mass_vector.end(), std::atof(line.substr(last, next - last).c_str()));
				last = next + 1;
			}
			mass_vector.insert(mass_vector.end(), std::atof(line.substr(last, next - last).c_str()));
			massMatrix.push_back(mass_vector);
			cnt++;
		}
		mass_file.close();
	}
}

void Simulator::GetConstraints() {
	// Read in Constraints
	std::ifstream positions_file;
	std::string line;
	std::ifstream constraints_file;
	constraints_file.open("constraints.txt");
	if (constraints_file.is_open())
	{
		int cnt = 1;
		while (std::getline(constraints_file, line))
		{
			// Create expression for each constraint
			exprtk::expression<double> expression;
			// Register all variables to every contstrain expression we have
			expression.register_symbol_table(this->symbol_table_t);
			// Parse the read in line and place it into expression
			this->parser_t.compile(line, expression);
			// We should now be good to go for calculations!
			this->C.push_back(expression);
		}
		constraints_file.close();
	}

	// Check that initial conditions satisfy C(x) = 0
	for (exprtk::expression<double> ex : this->C) {
		if (ex.value() != 0) {
			std::cout << "ERROR::C_MATRIX::INIT_CONDITION::" << ex.value() << "::NOT::EQUAL::TO::0" << std::endl;
		}
	}
}

void Simulator::CalculateCDot() {
	// Read in Constraints
	std::ifstream positions_file;
	std::string line;
	std::ifstream constraints_dot_file;
	constraints_dot_file.open("constraints_dot.txt");
	if (constraints_dot_file.is_open())
	{
		int cnt = 1;
		while (std::getline(constraints_dot_file, line))
		{
			// Create expression for each constraint
			exprtk::expression<double> expression;
			// Register all variables to every contstrain expression we have
			expression.register_symbol_table(this->symbol_table_t);
			// Parse the read in line and place it into expression
			this->parser_t.compile(line, expression);
			// We should now be good to go for calculations!
			this->Cdot.push_back(expression);
		}
		constraints_dot_file.close();
	}

	// Check that initial conditions satisfy C(x) = 0
	for (exprtk::expression<double> ex : this->Cdot) {
		if (ex.value() != 0) {
			std::cout << "ERROR::C_DOT_MATRIX::INIT_CONDITION::" << ex.value() << "::NOT::EQUAL::TO::0" << std::endl;
		}
	}
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
	std::vector<std::vector<double>> j_transpose = this->Transpose(this->J);
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
	std::vector<std::vector<double>> linear_system = this->MatMult(this->MatMult(J, W), j_transpose);


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
	

	

	std::vector<double> a = this->MatVecMult(neg_j_dot, q_dot);
	std::vector<double> b = this->MatVecMult(W, Q);
	std::vector<double> c = this->MatVecMult(J, b);
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

	auto feedback = this->VecSub(feedback1, feedback2);
	std::vector<double> linear_solution = this->VecSub(a, c);
	//this->PrintVector(linear_solution, "Linear Solution");
	

	std::vector<double> lambda = this->LinearSolve(linear_system, linear_solution);

	auto const_force = this->MatVecMult(j_transpose, lambda);
	auto total_force = this->VecAdd(Q, const_force);

	this->position_derivatives = q_dot;	// q_dot = velocities
	this->velocity_derivatives = this->MatVecMult(W, total_force);
}

std::vector<std::vector<double>> Simulator::Transpose(std::vector<std::vector<double>> mat)
{

	std::vector<std::vector<double>> trans_vec(mat[0].size(), std::vector<double>());

	for (int i = 0; i < mat.size(); i++)
	{
		for (int j = 0; j < mat[i].size(); j++)
		{
			trans_vec[j].push_back(mat[i][j]);
		}
	}

	return trans_vec;
}

std::vector<std::vector<double>> Simulator::MatMult(std::vector<std::vector<double>> A, std::vector<std::vector<double>> B) {
	std::vector<std::vector<double>> result_mat;

	result_mat.resize(A.size());
	for (auto it = result_mat.begin(); it != result_mat.end(); ++it) {
		it->resize(B[0].size());
	}


	for (int i = 0; i < result_mat.size(); ++i) {
		for (int j = 0; j < result_mat[0].size(); ++j) {
			double val = 0.0f;
			for (int k = 0; k < A[0].size(); k++) {
				val += (A[i][k] * B[k][j]);
			}
			result_mat[i][j] = double(val);
		}
		
	}

	return result_mat;
}

std::vector<double> Simulator::MatVecMult(std::vector<std::vector<double>> A, std::vector<double> B) {
	std::vector<double> result_mat;

	for (int i = 0; i < A.size(); i++) {
		double val = 0.0;
		for (int j = 0; j < B.size(); j++) {
			val += A[i][j] * B[j];
		}
		result_mat.push_back(val);
	}

	return result_mat;
}

std::vector<double> Simulator::VecSub(std::vector<double> A, std::vector<double> B) {
	std::vector<double> result_mat;

	for (int i = 0; i < A.size(); i++) {
		result_mat.push_back(A[i] - B[i]);
	}

	return result_mat;
}

std::vector<double> Simulator::VecAdd(std::vector<double> A, std::vector<double> B) {
	std::vector<double> result_mat;

	for (int i = 0; i < A.size(); i++) {
		result_mat.push_back(A[i] + B[i]);
	}

	return result_mat;
}

std::vector<std::vector<double>> MatScalarMult(std::vector<std::vector<double>> mat, double scalar) {
	for (int i = 0; i < mat.size(); i++) {
		for (int j = 0; j < mat[0].size(); j++) {
			mat[i][j] *= scalar;
		}
	}
	return mat;
}

std::vector<double> Simulator::LinearSolve(std::vector<std::vector<double>> a, std::vector<double> b) {
	int N = a.size();
	std::vector<double> x(a.size());
	//x.resize(n);
	std::fill(x.begin(), x.end(), 0);
	//this->PrintMatrix(a, "A Before = ");
	//std::cout << b.size() << std::endl;
	for(int i = 0; i < a.size(); i++) {
		a[i].push_back(b[i]);
	}
	// Applying Gauss Elimination
	// to find the elements of diagonal matrix
	int i, j, k;
	for (j = 0; j < N - 1; j++)
	{
		for (i = j + 1; i < N; i++)
		{
			double temp = a[i][j] / a[j][j];

			for (k = 0; k < N + 1; k++)
				a[i][k] -= a[j][k] * temp;
		}
	}

	for (i = N - 1; i >= 0; i--)
	{
		double s = 0;
		for (j = i + 1; j < N; j++)
			s += a[i][j] * x[j];
		x[i] = (a[i][N] - s) / a[i][i];
	}
	return x;
}




//void Simulator::ProcessInput(double dt) {
//
//
//}

// Utility Functions
// -----------------
glm::vec3 Simulator::Function(glm::vec3 dir) {
	return glm::vec3(0.0f);
}

// Euler Solver (this maybe able to be void since using global particle system
std::vector<double> Simulator::Euler_step(double h) {
	this->EvalDerivative();


	auto pos = this->positions;
	auto vel = this->velocities;

	auto dp = this->position_derivatives;
	auto dv = this->velocity_derivatives;

	int cnt = 0;
	for (int i = 0; i < dp.size(); i++) {
		dp[i] *= h;
		dv[i] *= h;
	}
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
// Midpoint Solver
//glm::vec3 Simulator::Midpoint_Step(double h) {
//	return glm::vec3(0.0f);
//
//}
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

void Simulator::ProcessMouseInput(double x, double y) {
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
void Simulator::PrintVector(std::vector<double> vec, std::string name){
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
