#include "argv.hpp"

#include <catch2/catch.hpp>

namespace dr {

using namespace std::string_literals;

TEST_CASE("Argv -- split", "split") {
	std::vector<std::string> args = {
		"aap",
		"noot:=mies",
		"wim",
		"zus:=jet",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());
	CHECK(regular == std::vector{"aap"s, "wim"s});
	CHECK(ros == std::map<std::string, std::string>{{"noot", "mies"}, {"zus", "jet"}});
}

TEST_CASE("Argv -- splitEmpty", "splitEmpty") {
	auto [regular, ros] = splitCommandLine(0, nullptr);
	CHECK(regular.empty());
	CHECK(ros.empty());
}

TEST_CASE("Argv -- emptySource", "emptySource") {
	std::vector<std::string> args = {
		":=a",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());

	CHECK(regular.empty());
	CHECK(ros == std::map<std::string, std::string>{{"", "a"}});
}

TEST_CASE("Argv -- emptyTarget", "emptyTarget") {
	std::vector<std::string> args = {
		"a:=",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());

	CHECK(regular.empty());
	CHECK(ros == std::map<std::string, std::string>{{"a", ""}});
}

TEST_CASE("Argv -- emptyBoth", "emptyBoth") {
	std::vector<std::string> args = {
		":=",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());

	CHECK(regular.empty());
	CHECK(ros == std::map<std::string, std::string>{{"", ""}});
}

}
