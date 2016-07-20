#pragma once

#include <functional>
#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <dr_log/dr_log.hpp>

namespace dr {

/// Simple subscriber that remembers the last message received.
template<typename Message>
class Subscriber {
public:
	using Callback = std::function<bool (Message const &, ros::Time const &)>;

private:
	/// The cached message.
	Message message_;

	/// The time when the message arrived.
	ros::Time time_{0, 0};

	/// The internal subscriber used.
	ros::Subscriber subscriber_;

	/// The topic name.
	std::string topic_name_ = "";

	/// True if a message has been received.
	bool message_received_ = false;

	/// Signal to notify the wait function it can return.
	bool wait_done_;

	/// All synchronous wait operations started before this time should cancel themselves.
	ros::Time cancel_before_{0, 0};

	/// Vector of callbacks to invoke on the next message.
	std::vector<Callback> waiters_;

	/// Mutex to obtain when accesing the message.
	std::mutex message_mutex_;

	/// Mutex to obtain when accesing the callback list.
	std::recursive_mutex callback_mutex_;

public:
	Subscriber() {}

	/// Construct the subscriber.
	Subscriber(
		ros::NodeHandle & node,                  ///< The node handle to use for name resolution.
		std::string const & topic,               ///< The name of the topic to subscribe to.
		uint32_t queue_size = 10,                ///< The queue size for the wrapper subscriber.
		ros::TransportHints transport_hints = {} ///< The transport hints for the wrapped subscriber.
	) {
		connect(node, topic,queue_size,transport_hints);
	}

	/// Initialize the subscriber.
	void connect(ros::NodeHandle & node,      ///< The node handle to use for name resolution.
		std::string const & topic,               ///< The name of the topic to subscribe to.
		uint32_t queue_size = 10,                ///< The queue size for the wrapper subscriber.
		ros::TransportHints transport_hints = {} ///< The transport hints for the wrapped subscriber.
	) {
		subscriber_ = node.subscribe(topic, queue_size, &Subscriber::onMessage, this, transport_hints);
		topic_name_ = topic;
	}

	/// Get the last message received.
	Message const & message() const {
		if(!message_received_)
			DR_ERROR("No messages has been received, but the message is requested (topic: " << topic_name_ << ")");
		if(topic_name_ == "")
			DR_ERROR("Subscriber not initialized, but the message is requested.");
		return message_;
	}

	/// Returns time stamp.
	ros::Time const time() {
		return time_;
	}

	/// Check if the subscriber has a message.
	bool hasMessage() const {
		return message_received_;
	}

	/// Wait for a message to a arrive.
	bool wait(ros::Duration timeout = ros::Duration(0), ros::Rate rate = ros::Rate(10)) {
		wait_done_ = false;
		ros::Time start = ros::Time::now();

		while (timeout == ros::Duration(0) || ros::Time::now() < start + timeout) {
			ros::spinOnce();
			if (wait_done_) return true;
			if (ros::Time::now() <= cancel_before_) return false;
			if (!ros::ok()) return false;
			rate.sleep();
		}

		return wait_done_;
	}

	/// Register a callback to be called once when the next message arrives.
	void asyncWait(Callback callback) {
		std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
		waiters_.push_back(callback);
	};

	/// Cancel all pending wait operations (synchronous and asynchronous).
	void cancel() {
		std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
		cancel_before_ = ros::Time::now();
		waiters_.clear();
	}

private:
	/// Handle received messages.
	void onMessage(Message const & message) {
		std::lock_guard<std::mutex> lock(message_mutex_);

		message_          = message;
		time_             = ros::Time::now();
		message_received_ = true;
		wait_done_        = true;

		{
			std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
			for (unsigned int i = 0; i < waiters_.size(); ++i) {
				if (waiters_[i](message_, time_)) {
					waiters_.erase(waiters_.begin() + i);
					--i;
				}
			}
		}
	}

};

}
