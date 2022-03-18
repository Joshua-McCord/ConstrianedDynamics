#pragma once
#ifndef PARSER
#include <format>
#include <string>
#include <vector>
#include <iostream>
#include <exprtk/exprtk.hpp>
#include <regex>



class Parser {
public:

	void ParseFile(std::string fileName, std::vector<exprtk::expression<double>>* destination, exprtk::symbol_table<double> symbol_table_t);
	void ParseFile(std::string fileName, std::vector<std::pair<std::string, double>>* destination, exprtk::symbol_table<double> symbol_table_t, char dest);
	void ParseFile(std::string fileName, std::vector<std::vector<double>>* destination, exprtk::symbol_table<double> symbol_table_t);

private:
	exprtk::parser<double> parser;

	void CheckForErrors(std::vector<exprtk::expression<double>>* destination);

};

#endif // !PARSER
