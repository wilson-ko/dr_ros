#pragma once
#include "dispatch.hpp"

#include <ros/ros.h>
#include <dr_log/dr_log.hpp>
#include <dr_param/param.hpp>

#include <vector>
#include <algorithm>
#include <utility>



namespace dr {

/// Shorthand to get a ServiceEvent type for a service.
template<typename Service>
using ServiceEvent = ros::ServiceEvent<typename Service::Request, typename Service::Response>;

/// A ROS node wrapper with some utility functions.
class Node : ros::NodeHandle {
private:
	/// Run prefix for saving log files and logged data.
	std::string run_prefix_;

	/// Node prefix for saving log files and logged data.
	std::string node_prefix_;

public:
	/// Construct a node.
	Node();

	/// Get the run prefix for saving log files and logged data.
	std::string runPrefix();

	/// Get the node prefix for saving log files and logged data.
	std::string nodePrefix();

protected:
	/// Get a parameter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	T getParam(std::string const & name) {
		return dr::getParam<T>(*this, name);
	}

	/// Get a parameter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	T getParam(std::string const & name, T const & fallback, bool warn = true) {
		return dr::getParam<T>(*this, name, fallback, warn);
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	std::vector<T> getParamList(std::string const & name) {
		return dr::getParamList<T>(*this, name);
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	std::vector<T> getParamList(std::string const & name, T const & fallback, bool warn = true) {
		return dr::getParamList<T>(*this, name, fallback, warn);
	}

	/// Search for a parameter up the namespace hierarchy and return it's value.
	template<typename T>
	T searchParam(std::string const & name, T const & fallback, bool warn = true) {
		std::string key;
		if (!ros::NodeHandle::searchParam(name, key)) return fallback;
		return getParam<T>(key, fallback, warn);
	}

	/// Dispatch a callback for later invocation.
	template<typename F>
	void dispatch(F && f) {
		dr::dispatch(*this, std::forward<F>(f));
	}

public:
	/// Check if ROS thinks we should keep running.
	bool ok() {
		return ros::NodeHandle::ok();
	}
};

}
