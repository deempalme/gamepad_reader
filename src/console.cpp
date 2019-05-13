#include "gamepad_reader/console.h"
#include "ui_console.h"

Console::Console(int milliseconds_frequency, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::Console),
  timer_(nullptr),
  gamepads_(nullptr)
{
  ui->setupUi(this);

  set_initial_console();

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(reload_data()));
  timer_->start(milliseconds_frequency);
}

Console::~Console(){
  delete ui;
  delete timer_;
}

void Console::set_gamepads(const std::vector<gamepad::GamepadData> *const gamepads){
  gamepads_ = gamepads;
}

void Console::keyPressEvent(QKeyEvent */*event*/){
  //    emit readAll();
}

void Console::set_initial_console(){
  ui->b_a->setStyleSheet("background:none;");
  ui->b_b->setStyleSheet("background:none;");
  ui->b_x->setStyleSheet("background:none;");
  ui->b_y->setStyleSheet("background:none;");
  ui->b_lsb->setStyleSheet("background:none;");
  ui->b_rsb->setStyleSheet("background:none;");
  ui->b_lb->setStyleSheet("background:none;");
  ui->b_rb->setStyleSheet("background:none;");
  ui->b_d_down->setStyleSheet("background:none;");
  ui->b_d_up->setStyleSheet("background:none;");
  ui->b_d_right->setStyleSheet("background:none;");
  ui->b_d_left->setStyleSheet("background:none;");
  ui->b_start->setStyleSheet("background:none;");
  ui->b_back->setStyleSheet("background:none;");
  ui->b_guide->setStyleSheet("background:none;");
  ui->b_clutch->setStyleSheet("background:none;");
  ui->b_brake->setStyleSheet("background:none;");
  ui->b_gas->setStyleSheet("background:none;");
}

void Console::reload_data(){
  emit read_all(-1);

  if(gamepads_){
    for(uint i = 0; i < gamepads_->size(); i++){
      ui->t_angle->setText(QString("%1°").arg(gamepads_->at(i).steering * RADtoDEGREES, 1, 'f', 2, 0));
      ui->t_clutch->setText(QString("%1%").arg(gamepads_->at(i).clutch * 100, 1, 'f', 1, 0));
      ui->t_brake->setText(QString("%1%").arg(gamepads_->at(i).brake * 100, 1, 'f', 1, 0));
      ui->t_gas->setText(QString("%1%").arg(gamepads_->at(i).gas * 100, 1, 'f', 1, 0));

      if(gamepads_->at(i).A)
        ui->b_a->setStyleSheet("background:url(:/images/b_button) no-repeat;");
      else
        ui->b_a->setStyleSheet("background:none;");

      if(gamepads_->at(i).B)
        ui->b_b->setStyleSheet("background:url(:/images/b_button) no-repeat;");
      else
        ui->b_b->setStyleSheet("background:none;");

      if(gamepads_->at(i).X)
        ui->b_x->setStyleSheet("background:url(:/images/b_button) no-repeat;");
      else
        ui->b_x->setStyleSheet("background:none;");

      if(gamepads_->at(i).Y)
        ui->b_y->setStyleSheet("background:url(:/images/b_button) no-repeat;");
      else
        ui->b_y->setStyleSheet("background:none;");

      if(gamepads_->at(i).RB)
        ui->b_rb->setStyleSheet("background:url(:/images/b_rb) no-repeat;");
      else
        ui->b_rb->setStyleSheet("background:none;");

      if(gamepads_->at(i).LB)
        ui->b_lb->setStyleSheet("background:url(:/images/b_lb) no-repeat;");
      else
        ui->b_lb->setStyleSheet("background:none;");

      if(gamepads_->at(i).RSB)
        ui->b_rsb->setStyleSheet("background:url(:/images/b_rsb) no-repeat;");
      else
        ui->b_rsb->setStyleSheet("background:none;");

      if(gamepads_->at(i).LSB)
        ui->b_lsb->setStyleSheet("background:url(:/images/b_lsb) no-repeat;");
      else
        ui->b_lsb->setStyleSheet("background:none;");

      if(gamepads_->at(i).start)
        ui->b_start->setStyleSheet("background:url(:/images/b_start) no-repeat;");
      else
        ui->b_start->setStyleSheet("background:none;");

      if(gamepads_->at(i).back)
        ui->b_back->setStyleSheet("background:url(:/images/b_back) no-repeat;");
      else
        ui->b_back->setStyleSheet("background:none;");

      if(gamepads_->at(i).guide)
        ui->b_guide->setStyleSheet("background:url(:/images/b_guide) no-repeat;");
      else
        ui->b_guide->setStyleSheet("background:none;");

      if(gamepads_->at(i).up)
        ui->b_d_up->setStyleSheet("background:url(:/images/b_d_up) no-repeat;");
      else
        ui->b_d_up->setStyleSheet("background:none;");

      if(gamepads_->at(i).down)
        ui->b_d_down->setStyleSheet("background:url(:/images/b_d_down) no-repeat;");
      else
        ui->b_d_down->setStyleSheet("background:none;");

      if(gamepads_->at(i).right)
        ui->b_d_right->setStyleSheet("background:url(:/images/b_d_right) no-repeat;");
      else
        ui->b_d_right->setStyleSheet("background:none;");

      if(gamepads_->at(i).left)
        ui->b_d_left->setStyleSheet("background:url(:/images/b_d_left) no-repeat;");
      else
        ui->b_d_left->setStyleSheet("background:none;");

      if(gamepads_->at(i).clutch > 0.0)
        ui->b_clutch->setStyleSheet("background:url(:/images/b_left_pedal) no-repeat;");
      else
        ui->b_clutch->setStyleSheet("background:none;");

      if(gamepads_->at(i).brake > 0.0)
        ui->b_brake->setStyleSheet("background:url(:/images/b_left_pedal) no-repeat;");
      else
        ui->b_brake->setStyleSheet("background:none;");

      if(gamepads_->at(i).gas > 0.0)
        ui->b_gas->setStyleSheet("background:url(:/images/b_right_pedal) no-repeat;");
      else
        ui->b_gas->setStyleSheet("background:none;");
    }
  }
}
