#pragma once

#include <ros/node_handle.h>
#include <ros/callback_queue_interface.h>

#include <boost/shared_ptr.hpp>

#include <utility>

namespace dr {

/// Wrap a functor in a ros::CallbackInterface.
template<typename Functor>
class RosCallbackWrapper : public ros::CallbackInterface {
	Functor functor;

public:
	RosCallbackWrapper(Functor && functor) : functor(std::forward<Functor>(functor)) {};

	virtual CallResult call() override {
		functor();
		return CallResult::Success;
	}
};

/// Dispatch a callback to a specified callback queue.
template<typename Functor>
void dispatch(ros::CallbackQueueInterface & queue, Functor && functor) {
	using Decayed = typename std::decay<Functor>::type;
	queue.addCallback(boost::make_shared<RosCallbackWrapper<Decayed>>(std::forward<Functor>(functor)));
}

/// Dispatch a callback to the default callback queue of a node.
template<typename Functor>
void dispatch(ros::NodeHandle & node, Functor && functor) {
	return dispatch(*node.getCallbackQueue(), functor);
}

}
