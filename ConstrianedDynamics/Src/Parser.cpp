#include "../Headers/Parser.h"



void Parser::ParseFile(std::string fileName, std::vector<exprtk::expression<double>>* destination, exprtk::symbol_table<double> symbol_table_t) {
	exprtk::parser<double> parser;

	std::string line;
	std::ifstream file;
	file.open(fileName);
	if (file.is_open())
	{
		int cnt = 1;
		while (std::getline(file, line))
		{
			// Create expression
			exprtk::expression<double> expression;
			// Register all variables to every expression
			expression.register_symbol_table(symbol_table_t);
			// Parse the read in line and place it into expression
			parser.compile(line, expression);

			destination->push_back(expression);
		}
		file.close();
	}
	// Check the initial conditions
	this->CheckForErrors(destination);
}

void Parser::ParseFile(std::string fileName, std::vector<std::pair<std::string, double>>* destination, exprtk::symbol_table<double> symbol_table_t, char dest) {
	std::ifstream file;
	file.open(fileName);
	std::string line;
	if (file.is_open())
	{
		int cnt = 1;
		while (std::getline(file, line))
		{
			std::string delimiter = ",";
			size_t last = 0;
			size_t next = 0;

			while ((next = line.find(delimiter, last)) != std::string::npos) {
				destination->push_back(
					std::pair<std::string, double>(
						std::regex_replace(std::format("{}x{}", dest, cnt), std::regex("^ +| +$|( ) +"), "$1"), // Regex Magic -> Trim Whitespace
						double(std::atof(line.substr(last, next - last).c_str()))
						)
				);
				last = next + 1;
			}
			destination->push_back(
				std::pair<std::string, double>(
					std::regex_replace(std::format("{}y{}", dest, cnt), std::regex("^ +| +$|( ) +"), "$1"), // Regex Magic -> Trim Whitespace
					double(std::atof(line.substr(last).c_str()))
					)
			);
			cnt++;
		}
		file.close();
	}

	// Add all positions to symbol table
	for (auto it = destination->begin(); it != destination->end(); ++it) {
		symbol_table_t.add_variable(it->first, it->second);
	}
}

void Parser::ParseFile(std::string fileName, std::vector<std::vector<double>>* destination, exprtk::symbol_table<double> symbol_table_t) {
	
	std::ifstream file;
	file.open(fileName);
	std::string line;
	std::vector<double> tmp_vector;
	if (file.is_open())
	{
		int cnt = 1;
		
		while (std::getline(file, line))
		{
			std::string delimiter = ",";
			size_t last = 0;
			size_t next = 0;

			tmp_vector.clear();
			while ((next = line.find(delimiter, last)) != std::string::npos) {
				tmp_vector.insert(tmp_vector.end(), std::atof(line.substr(last, next - last).c_str()));
				last = next + 1;
			}
			tmp_vector.insert(tmp_vector.end(), std::atof(line.substr(last, next - last).c_str()));
			destination->push_back(tmp_vector);
			cnt++;
		}
		file.close();
	}
}

void Parser::CheckForErrors(std::vector<exprtk::expression<double>>* destination) {
	for (exprtk::expression<double> ex : *destination) {
		if (ex.value() != 0) {
			std::cout << "ERROR::INIT_CONDITION::" << ex.value() << "::NOT::EQUAL::TO::0" << std::endl;
		}
	}
}