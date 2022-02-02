#include "../Headers/Parser.h"

void ParseFile(std::string fileName, std::vector<exprtk::expression<double>>* destination, exprtk::symbol_table<double> symbol_table_t) {
	// Read in Constraints
	std::ifstream positions_file;
	std::string line;
	std::ifstream file;
	file.open(fileName);
	if (file.is_open())
	{
		int cnt = 1;
		while (std::getline(file, line))
		{
			// Create expression for each constraint
			exprtk::expression<double> expression;
			// Register all variables to every contstraint expression we have
			expression.register_symbol_table(symbol_table_t);
			// Parse the read in line and place it into expression
			parser.compile(line, expression);
			// We should now be good to go for calculations!
			destination->push_back(expression);
		}
		file.close();
	}
	// Check that initial conditions satisfy C(x) = 0
	for (exprtk::expression<double> ex : *destination) {
		if (ex.value() != 0) {
			std::cout << "ERROR::INIT_CONDITION::" << ex.value() << "::NOT::EQUAL::TO::0" << std::endl;
		}
	}
}