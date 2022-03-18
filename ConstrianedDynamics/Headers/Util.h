//#pragma once
#include <format>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <format>
#include <string>
#include <string_view>
#include <fstream>
#include <sstream>
#include <exprtk/exprtk.hpp>

//namespace Logger {
//	void PrintMatrix(std::vector<std::vector<double>> mat, std::string name);
//	void PrintVector(std::vector<double> vec, std::string name);
//}
#ifndef PARSER
#define PARSER

namespace Parser {
	//typedef exprtk::parser<double>       parser_d;
	//static parser_d parser;

	void ParseFile(std::string fileName, std::vector<exprtk::expression<double>>* destination, exprtk::symbol_table<double> symbol_table_t);
}

#endif // !PARSER

