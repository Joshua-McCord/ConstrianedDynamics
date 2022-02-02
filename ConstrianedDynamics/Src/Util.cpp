#include "../Headers/Util.h"
//
//namespace Logger {
//	void PrintMatrix(std::vector<std::vector<double>> mat, std::string name) {
//		std::cout << name << " = " << std::endl;
//		std::cout << "[";
//		for (int i = 0; i < mat.size(); ++i) {
//			std::cout << "[";
//			for (int j = 0; j < mat[0].size(); ++j) {
//				std::cout << mat[i][j];
//				if (j != mat[0].size() - 1) {
//					std::cout << ", ";
//				}
//			}
//			std::cout << "]";
//			if (i != mat.size() - 1) {
//				std::cout << ",\n";
//			}
//		}
//		std::cout << "]" << std::endl;
//	}
//	void PrintVector(std::vector<double> vec, std::string name) {
//		std::cout << name << " = " << std::endl;
//		std::cout << "[";
//		for (int i = 0; i < vec.size(); ++i) {
//			std::cout << vec[i];
//			if (i != vec.size() - 1) {
//				std::cout << ", ";
//			}
//		}
//		std::cout << "]" << std::endl;
//	}
//}
//
namespace Parser {

//	void ParseFile(std::string fileName, std::vector<std::vector<exprtk::expression<double>>>* destination, exprtk::parser<double> parser_t, exprtk::symbol_table<double> symbol_table_t) {
//		std::vector<std::string> equation;
//		std::string line;
//		std::ifstream file;
//		file.open(fileName);
//		if (file.is_open())
//		{
//			int cnt = 1;
//			while (std::getline(file, line))
//			{
//				std::vector<exprtk::expression<double>> result;
//				std::stringstream s_stream(line); //create string stream from the string
//				while (s_stream.good()) {
//					std::string substr;
//					getline(s_stream, substr, ','); //get first string delimited by comma
//					// Create expression for each constraint
//					exprtk::expression<double> expression;
//					// Register all variables to every contstraint expression we have
//					expression.register_symbol_table(symbol_table_t);
//					// Parse the read in substr and place it into expression
//					parser_t.compile(substr, expression);
//					result.push_back(expression);
//				}
//				destination->push_back(result);
//			}
//			file.close();
//		}
//	}
}