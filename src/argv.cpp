#include "argv.hpp"

std::pair<std::vector<std::string>, std::map<std::string, std::string>> splitCommandLine(int argc, char const * const * argv) {
	std::vector<std::string> regular;
	std::map<std::string, std::string> ros;

	for (int i = 0; i < argc; ++i) {
		char const * sep = argv[i];
		while (static_cast<bool>(*sep) && !(sep[0] == ':' && (sep[1]) == '=')) { ++sep; }

		// If we found a seperator, this is a ROS mapping.
		if (static_cast<bool>(*sep)) {
			ros.insert({{argv[i], sep}, std::string{sep + 2}});
		} else {
			regular.emplace_back(argv[i]);
		}
	}

	return {
		std::move(regular),
		std::move(ros),
	};
}
