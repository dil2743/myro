#ifndef __READCSV__
#define __READCSV__
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <iostream>
std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename);
#endif