#ifndef CONSOLE_H
#define CONSOLE_H

#include "includes/definitions.h"

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
  explicit Console(QWidget *parent = 0);
  ~Console();

  void set_gamepads(const std::vector<GamepadData> *const gamepads);

private:
  Ui::Console *ui;

  void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
  void set_initial_console();

  QTimer *timer_;
  const std::vector<GamepadData> *gamepads_;

signals:
  void read_all(const int);

private slots:
  void reload_data();
};

#endif // CONSOLE_H
