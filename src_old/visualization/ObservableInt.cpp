// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#include "ObservableInt.h"

ObservableInt::ObservableInt(int initialValue, QObject *parent) : QObject(parent), m_value(initialValue) {
}

void ObservableInt::setValue(int value) {
	if (m_value != value) {
		m_value = value;
		emit valueChanged(m_value);
	}
}

//QSharedPointer<ObservableInt> extractObservableInt(QObject *object, const char *property) {
//	auto propertyHolder = QSharedPointer<ObservableInt>(new ObservableInt());
//	QObject::connect(object, property, propertyHolder.data(), &ObservableInt::setValue);
//	return propertyHolder;
//}
