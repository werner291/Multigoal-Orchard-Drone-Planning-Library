#pragma once

#include <cstddef>

template<typename T>
class QSharedPointer;

class QComboBox;
class QWidget;

QSharedPointer<QComboBox> createAppleIdComboBox(const size_t num_apples, QWidget *parent = nullptr);