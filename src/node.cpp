#include "node.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/date_time/time_facet.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>

#include <sstream>

#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

namespace dr {

namespace {

std::string namespaceToName(std::string const & name) {
	std::string result = name;
	if (boost::algorithm::starts_with(result, "/")) result.erase(0, 1);
	boost::algorithm::replace_all(result, "/", ".");
	return result;
}

std::string formatTime(boost::posix_time::ptime time, std::string const & format) {
	using namespace boost::posix_time;

	std::stringstream buffer;
	std::locale locale(buffer.getloc(), new time_facet(format.c_str()));
	buffer.imbue(locale);
	buffer << time;
	return buffer.str();
}

std::string getHomeDirectory(std::string const & fallback) {
	char const * homedir = std::getenv("HOME");
	if (homedir) return homedir;

	std::vector<std::uint8_t> buffer(256);
	uid_t uid = getuid();
	passwd info;
	passwd * result;

	while (true) {
		errno = 0;
		int error = getpwuid_r(uid, &info, reinterpret_cast<char *>(buffer.data()), buffer.size(), &result);

		if (error == 0) return result != nullptr ? info.pw_dir : fallback;

		if (errno == ERANGE && buffer.size() < 1024) {
			buffer.resize(buffer.size() * 2);
			continue;
		}

		break;
	}

	return fallback;
}

}

Node::Node() : ros::NodeHandle("~") {
	boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

	std::string node_name = namespaceToName(this->getNamespace());
	std::string fallback  = getHomeDirectory(".") + "/.ros/run/" + formatTime(now, "%Y-%m-%d/%H-%M-%S");
	run_prefix_           = searchParam<std::string>("run_prefix", fallback);
	node_prefix_          = run_prefix_   + "/" + node_name;
	std::string log_file  = node_prefix_  + "/" + node_name + ".log";

	boost::system::error_code error;
	boost::filesystem::create_directories(node_prefix_, error);

	setupLogging(log_file, node_name);
};

std::string Node::runPrefix() {
	return run_prefix_;
}

std::string Node::nodePrefix() {
	return node_prefix_;
}

}
