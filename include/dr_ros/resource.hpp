#pragma once
#include <string>

namespace dr {

/// Convert a ROS package URL or a regular file URL to a file path.
/**
 * If the input does not contain an URI scheme it is treated as a relative file path and returned without modification.
 *
 * \throws if an unsupported URI scheme is encountered.
 */
std::string resolveResourceUrl(std::string const & url);

}
