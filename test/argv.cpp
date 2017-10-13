#include "argv.hpp"

#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

using namespace std::string_literals;

TEST(Argv, split) {
	std::vector<std::string> args = {
		"aap",
		"noot:=mies",
		"wim",
		"zus:=jet",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());
	ASSERT_EQ(regular, (std::vector{"aap"s, "wim"s}));
	ASSERT_EQ(ros, (std::map<std::string, std::string>{{"noot", "mies"}, {"zus", "jet"}}));
}

TEST(Argv, splitEmpty) {
	auto [regular, ros] = splitCommandLine(0, nullptr);
	ASSERT_TRUE(regular.empty());
	ASSERT_TRUE(ros.empty());
}

TEST(Argv, emptySource) {
	std::vector<std::string> args = {
		":=a",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());

	ASSERT_TRUE(regular.empty());
	ASSERT_EQ(ros, (std::map<std::string, std::string>{{"", "a"}}));
}

TEST(Argv, emptyTarget) {
	std::vector<std::string> args = {
		"a:=",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());

	ASSERT_TRUE(regular.empty());
	ASSERT_EQ(ros, (std::map<std::string, std::string>{{"a", ""}}));
}

TEST(Argv, emptyBoth) {
	std::vector<std::string> args = {
		":=",
	};
	std::vector<char const *> pointers;
	for (std::string const & str : args) pointers.push_back(str.data());

	auto [regular, ros] = splitCommandLine(args.size(), pointers.data());

	ASSERT_TRUE(regular.empty());
	ASSERT_EQ(ros, (std::map<std::string, std::string>{{"", ""}}));
}

}
