#ifndef GAMEPADREADER_H
#define GAMEPADREADER_H

#include "includes/definitions.h"

#include <QObject>
#include <QDebug>
#include <SDL2/SDL.h>
#include <SDL2/SDL_gamecontroller.h>
#include <vector>
#include <assert.h>

class GamepadReader : public QObject
{
  Q_OBJECT
public:
  GamepadReader();
  ~GamepadReader();

  const std::vector<GamepadData> *const get_gamepad_data();
  const GamepadData *const get_gamepad_data(const int index);

public slots:
  void read_gamepad(const int index = -1);

private:
  std::vector<SDL_Joystick*> joysticks_;
  int map_id_;

  std::vector<GamepadData> gamepads_;
};

#endif // GAMEPADREADER_H
