#ifndef GAMEPADREADER_H
#define GAMEPADREADER_H

#include "gamepad_reader/definitions.h"

#include <QObject>
#include <QDebug>
#include <SDL2/SDL.h>
#include <SDL2/SDL_gamecontroller.h>
#include <vector>
#include <iostream>

#ifdef ROSSY_RUNNER
#include <ros/ros.h>
#include "gamepad_reader/gamepads.h"
#include "gamepad_reader/logitech_g920.h"

namespace gamepad {
  typedef gamepad_reader::gamepads controllers;
  typedef gamepad_reader::logitech_g920 logitech;
}
#endif

namespace gamepad {
  class GamepadReader : public QObject
  {
    Q_OBJECT
  public:
    GamepadReader();
    ~GamepadReader();

    const std::vector<GamepadData> *get_gamepad_data() const;
    const GamepadData *get_gamepad_data(const std::size_t index) const;
    void print_message(const std::string &message,
                       gamepad::Message type = gamepad::Message::Normal);

  public slots:
    void read_gamepad(const int index = -1);

  private:
    std::vector<SDL_Joystick*> joysticks_;
    int map_id_;

    std::vector<GamepadData> gamepads_;

#ifdef ROSSY_RUNNER
    ros::NodeHandle *handler_;
    ros::Publisher *output_;
#endif
  };
}
#endif // GAMEPADREADER_H
