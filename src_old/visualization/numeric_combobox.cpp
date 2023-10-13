
#include "numeric_combobox.h"

#include <QComboBox>
#include <QSharedPointer>
#include <QString>

#include <cstddef>

QSharedPointer<QComboBox> createAppleIdComboBox(const size_t num_apples, QWidget *parent)
{

	auto comboBox = QSharedPointer<QComboBox>(new QComboBox(parent));

	for (size_t i = 0; i < num_apples; i++)
	{
		comboBox->addItem(QString::number(i));
	}

	return comboBox;
}