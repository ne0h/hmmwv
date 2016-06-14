#include "joystickevent.hpp"

JoystickEvent::JoystickEvent(std::vector<bool> buttons,
		std::vector<short> axis)
	: m_buttons(buttons), m_axis(axis) {

}

JoystickEvent::~JoystickEvent() {

}

std::vector<bool> JoystickEvent::getButtons() {
	return m_buttons;
}

std::vector<short> JoystickEvent::getAxis() {
	return m_axis;
}
