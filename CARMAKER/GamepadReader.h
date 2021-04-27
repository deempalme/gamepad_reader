#ifndef GAMEPADREADER_H
#define GAMEPADREADER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_gamecontroller.h>
#include <iostream>
#include <vector>
#include <assert.h>

#include "definitions.h"

class GamepadReader
{
public:
  GamepadReader();
  ~GamepadReader();

  const std::vector<GamepadData> *const get_gamepad_data();
  const GamepadData *const get_gamepad_data(const int index);
  void read_gamepad(const int index = -1);

private:
  std::vector<SDL_Joystick*> joysticks_;
  int map_id_;

  std::vector<GamepadData> gamepads_;
};

#endif // GAMEPADREADER_H
