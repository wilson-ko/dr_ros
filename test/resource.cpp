#include "resource.hpp"

#include <ros/package.h>

#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(ResourceTest, packageUrl) {
	ASSERT_EQ(ros::package::getPath("dr_ros") + "/test.file", resolveResourceUrl("package://dr_ros/test.file"));
}

TEST(ResourceTest, localFileUrl) {
	ASSERT_EQ("/test.file", resolveResourceUrl("file:///test.file"));
}

TEST(ResourceTest, relativeFileUrl) {
	ASSERT_EQ("foo/test.file", resolveResourceUrl("foo/test.file"));
}

TEST(ResourceTest, remoteFileUrl) {
	ASSERT_ANY_THROW(resolveResourceUrl("file://host/test.file"));
}

TEST(ResourceTest, unsupportedScheme) {
	ASSERT_ANY_THROW(resolveResourceUrl("http://example.com/test.file"));
}

}
