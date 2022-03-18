#include <vector>
#include <string>
#include <iostream>

class Logger {
public:

	void PrintMatrix(std::vector<std::vector<double>> mat, std::string name);
	void PrintVector(std::vector<double> vec, std::string name);

};