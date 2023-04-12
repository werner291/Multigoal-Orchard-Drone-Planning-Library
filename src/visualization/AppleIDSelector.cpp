// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#include "AppleIDSelector.h"

AppleIDSelector::AppleIDSelector(int numberOfApples, QWidget *parent) : QWidget(parent), m_appleID(0) {
	auto layout = new QVBoxLayout(this);

	m_appleIDComboBox = new QComboBox();
	for (int i = 0; i < numberOfApples; i++) {
		m_appleIDComboBox->addItem(QString("Apple ID: %1").arg(i));
	}

	connect(m_appleIDComboBox,
			QOverload<int>::of(&QComboBox::currentIndexChanged),
			this,
			&AppleIDSelector::onAppleIDChanged);

	layout->addWidget(m_appleIDComboBox);
}

int AppleIDSelector::appleID() const {
	return m_appleID;
}

void AppleIDSelector::setAppleID(int appleID) {
	if (m_appleID != appleID) {
		m_appleID = appleID;
		m_appleIDComboBox->setCurrentIndex(appleID);
		emit appleIDChanged(m_appleID);
	}
}

void AppleIDSelector::onAppleIDChanged(int index) {
	setAppleID(index);
}
