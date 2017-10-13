#pragma once
#include <map>
#include <string>
#include <vector>

/// Split your command line in non-ROS and ROS options.
std::pair<std::vector<std::string>, std::map<std::string, std::string>> splitCommandLine(int argc, char const * const * argv);
