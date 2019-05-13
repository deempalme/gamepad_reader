
#ifdef ROSSY_RUNNER
#include <ros/ros.h>
#endif

#include "gamepad_reader/console.h"
#include <QApplication>
#include <QObject>

#include "gamepad_reader/GamepadReader.h"

int main(int argc, char *argv[])
{
#ifdef ROSSY_RUNNER
  // intiialize ros node
  ros::init(argc, argv, "gamepad_reader");
#endif

  gamepad::GamepadReader gamepad;

  QApplication a(argc, argv);
  Console GUI(20/*milliseconds delayment between frames*/);
  GUI.show();

  GUI.set_gamepads(gamepad.get_gamepad_data());

  QObject::connect(&GUI, SIGNAL(read_all(const int)), &gamepad, SLOT(read_gamepad(const int)));

  return a.exec();
}
