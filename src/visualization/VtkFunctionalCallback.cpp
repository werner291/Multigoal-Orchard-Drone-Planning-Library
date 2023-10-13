// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#include "VtkFunctionalCallback.h"

vtkFunctionalCallback *vtkFunctionalCallback::New() {
	return new vtkFunctionalCallback;
}

void vtkFunctionalCallback::Execute(vtkObject *caller, unsigned long eventId, void *) {
	if (eventId == event_id) {
		callback();
	}
}

void vtkFunctionalCallback::setEventId(unsigned long eventId) {
	event_id = eventId;
}