// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_VTKFUNCTIONALCALLBACK_H
#define MGODPL_VTKFUNCTIONALCALLBACK_H

#include <vtkCommand.h>
#include <functional>
#include <vtkNew.h>
#include <vtkSmartPointer.h>

/**
 * A vtkTimerCommand that calls the given callback when the timer fires, to allow the use of lambdas as callbacks in vtk.
 *
 * Do not forget to call setCallback() to set the callback to call when the timer fires.
 */
class vtkFunctionalCallback : public vtkCommand {
	/// The callback to call when the timer fires.
	std::function<void()> callback;
	unsigned long event_id{};
public:
	void setEventId(unsigned long eventId);

private:

	/// Constructor is private, use the static method New (or vtkNew) to create a new instance.
	vtkFunctionalCallback() = default;
public:

	/// Create a new vtkFunctionalCallback instance (usually through vtkNew).
	static vtkFunctionalCallback* New();

	/// Execute the callback. This simply calls the callback that was set earlier.
	virtual void Execute(vtkObject* caller,
						 unsigned long eventId,
						 void* vtkNotUsed(callData));

	/// Set the callback to call when the timer fires.
	void setCallback(const std::function<void()> &cb) {
		vtkFunctionalCallback::callback = cb;
	}
};

inline vtkSmartPointer<vtkFunctionalCallback> lambaCb(std::function<void()> cb) {

	// This is a bit hacky, but we need to keep the callbacks alive, so we store them in a static vector.
	static std::vector<vtkSmartPointer<vtkFunctionalCallback>> callbacks;

	vtkNew<vtkFunctionalCallback> cb_;
	cb_->setCallback(cb);

	callbacks.push_back(cb_);

	return cb_;
}

#endif //MGODPL_VTKFUNCTIONALCALLBACK_H
