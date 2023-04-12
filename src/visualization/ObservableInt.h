// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#ifndef NEW_PLANNERS_OBSERVABLEINT_H
#define NEW_PLANNERS_OBSERVABLEINT_H


#include <QObject>
#include <QSharedPointer>

class ObservableInt : public QObject {
Q_OBJECT
	Q_PROPERTY(int value READ value WRITE setValue NOTIFY valueChanged)

public:
	explicit ObservableInt(int initialValue = 0, QObject *parent = nullptr);

	[[nodiscard]] int value() const {
		return m_value;
	}

	void setValue(int value);

signals:

	void valueChanged(int value);

private:
	int m_value;
};

//QSharedPointer<ObservableInt> extractObservableInt(QObject *object, const char *property);


#endif //NEW_PLANNERS_OBSERVABLEINT_H
