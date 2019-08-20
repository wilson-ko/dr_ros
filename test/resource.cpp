#include "resource.hpp"

#include <ros/package.h>

#include <catch2/catch.hpp>

namespace dr {

TEST_CASE("ResourceTest -- packageUrl", "packageUrl") {
	REQUIRE(ros::package::getPath("dr_ros") + "/test.file" == resolveResourceUrl("package://dr_ros/test.file"));
}

TEST_CASE("ResourceTest -- localFileUrl", "localFileUrl") {
	REQUIRE("/test.file" == resolveResourceUrl("file:///test.file"));
}

TEST_CASE("ResourceTest -- relativeFileUrl", "relativeFileUrl") {
	REQUIRE("foo/test.file" == resolveResourceUrl("foo/test.file"));
}

TEST_CASE("ResourceTest -- remoteFileUrl", "remoteFileUrl") {
	REQUIRE_THROWS(resolveResourceUrl("file://host/test.file"));
}

TEST_CASE("ResourceTest -- unsupportedScheme", "unsupportedScheme") {
	REQUIRE_THROWS(resolveResourceUrl("http://example.com/test.file"));
}

}
