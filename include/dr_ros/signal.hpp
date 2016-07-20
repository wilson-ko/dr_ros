#pragma once

namespace dr {
	/// Set the SIGINT handler to one that closes STDIN and calls ros::shutdown().
	int fixSigInt();

}
