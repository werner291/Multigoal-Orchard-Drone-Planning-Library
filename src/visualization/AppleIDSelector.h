// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#ifndef NEW_PLANNERS_APPLEIDSELECTOR_H
#define NEW_PLANNERS_APPLEIDSELECTOR_H


#include <QWidget>
#include <QComboBox>
#include <QVBoxLayout>

class AppleIDSelector : public QWidget {
Q_OBJECT
	Q_PROPERTY(int appleID READ appleID WRITE setAppleID NOTIFY appleIDChanged)

public:
	explicit AppleIDSelector(int numberOfApples, QWidget *parent = nullptr);

	int appleID() const;

	void setAppleID(int appleID);

signals:

	void appleIDChanged(int appleID);

private slots:

	void onAppleIDChanged(int index);

private:
	QComboBox *m_appleIDComboBox;
	int m_appleID;
};

#endif //NEW_PLANNERS_APPLEIDSELECTOR_H
