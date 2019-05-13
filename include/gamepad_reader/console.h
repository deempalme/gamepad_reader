#ifndef CONSOLE_H
#define CONSOLE_H

#include "gamepad_reader/definitions.h"

#include <QMainWindow>
#include <QKeyEvent>
#include <QTimer>
#include <QDebug>
#include <vector>

namespace Ui {
class Console;
}

class Console : public QMainWindow
{
  Q_OBJECT

public:
  explicit Console(int milliseconds_frequency = 20, QWidget *parent = nullptr);
  ~Console() override;

  void set_gamepads(const std::vector<gamepad::GamepadData> *const gamepads);

private:
  Ui::Console *ui;

  void keyPressEvent(QKeyEvent */*event*/) Q_DECL_OVERRIDE;
  void set_initial_console();

  QTimer *timer_;
  const std::vector<gamepad::GamepadData> *gamepads_;

signals:
  void read_all(const int);

private slots:
  void reload_data();
};

#endif // CONSOLE_H
