#include "joystick.hpp"
#include <stdexcept>

Joystick::Joystick() :
	m_initialized(false) {
}

Joystick::~Joystick() {
}

bool Joystick::init() {
	SDL_Init(SDL_INIT_JOYSTICK);
	int num_joysticks = SDL_NumJoysticks();
	if (num_joysticks < 1) {
		throw std::runtime_error("No joysticks found!");
	}

	if(!num_joysticks) {
		return false;
	}

	// take first device that appears to be a joystick
	// and no strange vbox device for example...
	// ...which means more than 2 axes
	for (int i = 0; i < num_joysticks; i++) {
		SDL_Joystick *tmpStick = SDL_JoystickOpen(i);
		if (SDL_JoystickNumAxes(tmpStick) > 2) {
			m_joystick = tmpStick;
			m_name = SDL_JoystickName(i);
			break;
		}
	}

	SDL_JoystickEventState(SDL_ENABLE);
	SDL_JoystickEventState(SDL_QUERY);
	m_initialized = true;
	return true;
}

std::string Joystick::getName() {
	if(!m_initialized) {
		throw std::runtime_error("not initialized!");
	}
	return m_name;
}

JoystickEvent Joystick::getEvent() {
	if(!m_initialized) {
		throw std::runtime_error("not initialized!");
	}

	SDL_Event event;
	std::vector<bool>  buttons;
	std::vector<short> axis;

	SDL_JoystickUpdate();
	
	// get keys
	for (int i = 0; i < SDL_JoystickNumButtons(m_joystick); i++) {
		unsigned int n = SDL_JoystickGetButton(m_joystick, i);
		buttons.push_back((n == 1) ? true : false);
	}
	
	// get axis
	for (int i = 0; i < SDL_JoystickNumAxes(m_joystick); i++) {
		signed short a = SDL_JoystickGetAxis (m_joystick, i);
		axis.push_back(a);
	}

	return JoystickEvent(buttons, axis);
}
