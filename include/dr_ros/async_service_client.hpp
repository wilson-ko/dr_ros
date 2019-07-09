#pragma once

#include "service_client.hpp"

#include <string>
#include <future>

namespace dr {

/// Wrapper around ros::ServiceClient for easy presistent, reconnecting services.
template<typename Service>
class AsyncServiceClient {
	using Request  = typename Service::Request;
	using Response = typename Service::Response;
	using Callback = std::function<void (std::optional<Response const &>)>;

	ServiceClient<Service> client_;
	std::thread thread_;

public:
	/// Construct a service client without connecting it to a service.
	AsyncServiceClient() {}

	/// Construct a service client and connect it to a service.
	AsyncServiceClient(ros::NodeHandle & node, std::string const & name, bool wait = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		connect(node, name, wait, timeout, verbose);
	}

	/// Destruct the client, joining the background thread if it is running.
	~AsyncServiceClient() {
		if (thread_.joinable()) thread_.join();
	}

	/// Check if the service is connected.
	bool isConnected() const {
		return client_.isConnected();
	}

	/// Check if the service is available.
	bool isAvailable() {
		return client_.isAvailable();
	}

	/// Connect to a service by name.
	bool connect(ros::NodeHandle & node, std::string name, bool wait = true, ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		return client_.connect(node, name, wait, timeout, verbose);
	}

	/// Wait for the service to connect.
	bool wait(ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		return client_.wait(timeout, verbose);
	}

	/// Asynchronously call the service.
	void call(Request const & request, Callback const & callback, bool reconnect = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		thread_ = std::thread([request, callback, reconnect, timeout, verbose] () {
			std::promise<Response> promise;
			try {
				Response response;
				client(request, response, reconnect, timeout, verbose);
				callback(response);
			} catch (...) {
				callback(std::nullopt);
			}
		});
	}

};

}
