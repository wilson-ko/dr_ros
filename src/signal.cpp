#include <iostream>
#include <cstdio>

#include <csignal>

#include <ros/ros.h>

#include "signal.hpp"

namespace dr {

namespace {

	void handleSigInt(int sig, siginfo_t * siginfo, void * context) {
		(void) sig;
		(void) context;
		(void) siginfo;
		std::fclose(stdin); //NOLINT
		ros::shutdown();
	}

} //namespace

int fixSigInt() {
	struct sigaction sigaction_data{};
	sigemptyset(&sigaction_data.sa_mask);
	sigaction_data.sa_handler   = nullptr;
	sigaction_data.sa_flags     = SA_SIGINFO;
	sigaction_data.sa_sigaction = &handleSigInt;
	return sigaction(SIGINT, &sigaction_data, nullptr);
}

} //namespace dr
