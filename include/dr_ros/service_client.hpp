#pragma once

#include <string>

#include <ros/ros.h>
#include <dr_log/dr_log.hpp>

namespace dr {

class ServiceError : public std::runtime_error {
	using std::runtime_error::runtime_error;
};

/// Wrapper around ros::ServiceClient for easy presistent, reconnecting services.
template<typename Service>
class ServiceClient {
	using Request  = typename Service::Request;
	using Response = typename Service::Response;

	/// The node to use for connecting to services.
	ros::NodeHandle * node_ = nullptr;

	/// The internal service client.
	ros::ServiceClient client_;

	/// The name of the service.
	std::string name_;

public:
	/// Construct a service client without connecting it to a service.
	ServiceClient() {}

	/// Construct a service client and connect it to a service.
	ServiceClient(ros::NodeHandle & node, std::string const & name, bool wait = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		connect(node, name, wait, timeout, verbose);
	}

	/// Check if the service is connected.
	bool isConnected() const {
		return client_.isValid();
	}

	/// Check if the service is available.
	bool isAvailable() {
		return client_.exists();
	}

	/// Connect to a service by name.
	bool connect(ros::NodeHandle & node, std::string name, bool wait = true, ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		node_   = &node;
		name_   = name;
		client_ = node_->serviceClient<Request, Response>(name_, true);
		if (wait) return this->wait(timeout, verbose);
		return client_.isValid();
	}

	/// Reconnect to the current service if the connection was dropped.
	/**
	 * Does nothing if the service is still connected.
	 */
	bool reconnect(ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		if (client_.isValid()) return true;
		if (verbose) DR_WARN("Lost connection to service `" + name_ + "'. Reconnecting.");
		client_ = node_->serviceClient<Request, Response>(name_, true);
		return wait(timeout, verbose);
	}

	/// Wait for the service to connect.
	bool wait(ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		if (verbose) DR_INFO("Waiting for service `" << name_ << "'.");
		return client_.waitForExistence(timeout);
	}

	/// Call the service and return the response.
	/**
	 * \throws ServiceError when the service call fails.
	 */
	Response operator() (Request const & request, bool reconnect = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		Response response;
		if (reconnect) this->reconnect(timeout, verbose);
		if (!client_.call(request, response)) throw ServiceError("Failed to call service `" + name_ + "'.");
		return response;
	}

	/// Call the service.
	bool operator() (Request const & request, Response & response, bool reconnect = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		if (reconnect) this->reconnect(timeout, verbose);
		return client_.call(request, response);
	}

	/// Call the service.
	bool operator() (Service & service, bool reconnect = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		if (reconnect) this->reconnect(timeout, verbose);
		return client_.call(service);
	}
};

}
