#include "includes/console.h"
#include <QApplication>
#include <QObject>

#include "includes/GamepadReader.h"

int main(int argc, char *argv[])
{
  GamepadReader gamepad;

  QApplication a(argc, argv);
  Console GUI;
  GUI.show();

  GUI.set_gamepads(gamepad.get_gamepad_data());

  QObject::connect(&GUI, SIGNAL(read_all(const int)), &gamepad, SLOT(read_gamepad(const int)));

  return a.exec();
}
