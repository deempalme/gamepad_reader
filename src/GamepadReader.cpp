#include "gamepad_reader/GamepadReader.h"

namespace gamepad {
  GamepadReader::GamepadReader() :
    joysticks_(0, nullptr),
    map_id_(0),
    gamepads_(0)
  {

#ifdef ROSSY_RUNNER
    handler_ = new ros::NodeHandle;
    output_ = new ros::Publisher(handler_->advertise<gamepad::controllers>("/gamepads", 1));
#endif

    if(SDL_Init(SDL_INIT_GAMECONTROLLER) != 0){
      print_message("Unable to initialize SDL: " + std::string(SDL_GetError()),
                    gamepad::Message::Error);
      return;
    }

    GamepadData control;
    if((map_id_ = SDL_GameControllerAddMappingsFromFile("control_maps/logitech_g920_map.txt")) < 0)
      print_message("Error when reading the control map file: " + std::string(SDL_GetError()),
                    gamepad::Message::Error);

    if(SDL_NumJoysticks() > 0){
      for(int i = 0; i < SDL_NumJoysticks(); i++){
        SDL_Joystick *joy = SDL_JoystickOpen(i);

        if(joy){
          print_message("Joystick with index \'" + std::to_string(i)
                        + "\' is a compatible controller, named \'"
                        + std::string(SDL_JoystickNameForIndex(i)) + "\'");
          joysticks_.push_back(joy);
          gamepads_.push_back(control);
        }else
          print_message("Could not open gamecontroller " + std::to_string(i) + ':'
                        + std::string(SDL_GetError()), gamepad::Message::Error);
      }
    }else{
      print_message("There are no controllers detected", gamepad::Message::Error);
    }
  }

  GamepadReader::~GamepadReader(){
#ifdef ROSSY_RUNNER
    delete handler_;
    delete output_;
#endif

    for(uint i = 0; i < joysticks_.size(); i++)
      if(joysticks_[i])
        SDL_JoystickClose(joysticks_[i]);

    SDL_Quit();
  }

  const std::vector<GamepadData> *GamepadReader::get_gamepad_data() const{
    return &gamepads_;
  }

  const GamepadData *GamepadReader::get_gamepad_data(const std::size_t index) const{
    return &gamepads_[index];
  }

  void GamepadReader::print_message(const std::string &message, gamepad::Message type){
    switch(static_cast<unsigned int>(type)){
      case static_cast<unsigned int>(gamepad::Message::Error):
        std::cout << "\n\033[1;41m Error: \033[0;1;38;5;174m ";
      break;
      case static_cast<unsigned int>(gamepad::Message::Warning):
        std::cout << "\n\033[1;30;48;5;11m Warning: \033[0;1;38;5;229m ";
      break;
      case static_cast<unsigned int>(gamepad::Message::Attention):
        std::cout << "\n\033[1;30;46m Attention: \033[0;1;38;5;195m ";
      break;
      default:
        std::cout << "\n\033[1;30;42m Message received: \033[0;1;38;5;193m ";
      break;
    }
    std::cout << message << "\033[0m" << std::endl;
  }

  void GamepadReader::read_gamepad(const int index){
    std::size_t total{joysticks_.size()};

    if(total > 0){
      SDL_JoystickUpdate();

      uint8_t hat;
      int16_t axis;

      std::size_t initial{static_cast<std::size_t>(index)};

      if(index >= 0 && total > initial)
        total = initial + 1;
      else if(index < 0)
        initial = 0;

      for(; initial < total; initial++){
        //directional PAD:
        hat = SDL_JoystickGetHat(joysticks_[initial], 0);

        gamepads_[initial].up = false;
        gamepads_[initial].down = false;
        gamepads_[initial].left = false;
        gamepads_[initial].right = false;

        switch(hat){
          case SDL_HAT_UP:
            gamepads_[initial].up = true;
            break;
          case SDL_HAT_DOWN:
            gamepads_[initial].down = true;
            break;
          case SDL_HAT_LEFT:
            gamepads_[initial].left = true;
            break;
          case SDL_HAT_RIGHT:
            gamepads_[initial].right = true;
            break;
          case SDL_HAT_RIGHTUP:
            gamepads_[initial].right = true;
            gamepads_[initial].up = true;
            break;
          case SDL_HAT_RIGHTDOWN:
            gamepads_[initial].right = true;
            gamepads_[initial].down = true;
            break;
          case SDL_HAT_LEFTUP:
            gamepads_[initial].left = true;
            gamepads_[initial].up = true;
            break;
          case SDL_HAT_LEFTDOWN:
            gamepads_[initial].left = true;
            gamepads_[initial].down = true;
            break;
        }

        //brake
        axis = SDL_JoystickGetAxis(joysticks_[initial], 2);
        gamepads_[initial].brake = 1.0 - static_cast<double>(axis + 32768) / 65535.0;

        //gas
        axis = SDL_JoystickGetAxis(joysticks_[initial], 1);
        gamepads_[initial].gas = 1.0 - static_cast<double>(axis + 32768) / 65535.0;

        //clutch
        axis = SDL_JoystickGetAxis(joysticks_[initial], 3);
        gamepads_[initial].clutch = 1.0 - static_cast<double>(axis + 32768) / 65535.0;

        //steering
        axis = SDL_JoystickGetAxis(joysticks_[initial], 0);
        gamepads_[initial].steering = -static_cast<double>(axis + 32768) / 65535.0
                                      * _900degrees + _450degrees;

        if(gamepads_[initial].clutch == 0.5
           && gamepads_[initial].brake == 1.0
           && gamepads_[initial].gas == 1.0){
          gamepads_[initial].clutch = 0.0;
          gamepads_[initial].brake = 0.0;
          gamepads_[initial].gas = 0.0;
        }

        //buttons:
        gamepads_[initial].A     = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 0));
        gamepads_[initial].B     = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 1));
        gamepads_[initial].X     = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 2));
        gamepads_[initial].Y     = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 3));
        gamepads_[initial].RSB   = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 8));
        gamepads_[initial].LSB   = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 9));
        gamepads_[initial].RB    = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 4));
        gamepads_[initial].LB    = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 5));
        gamepads_[initial].back  = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 7));
        gamepads_[initial].start = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 6));
        gamepads_[initial].guide = static_cast<bool>(SDL_JoystickGetButton(joysticks_[initial], 10));
      }

#ifdef ROSSY_RUNNER
      gamepad::controllers controllers_content;
      gamepad::logitech controller_content;

      for(const GamepadData &controller : gamepads_){
        controller_content.A        = controller.A;
        controller_content.B        = controller.B;
        controller_content.X        = controller.X;
        controller_content.Y        = controller.Y;
        controller_content.RSB      = controller.RSB;
        controller_content.LSB      = controller.LSB;
        controller_content.RB       = controller.RB;
        controller_content.LB       = controller.LB;
        controller_content.back     = controller.back;
        controller_content.start    = controller.start;
        controller_content.guide    = controller.guide;

        controller_content.up       = controller.up;
        controller_content.down     = controller.down;
        controller_content.left     = controller.left;
        controller_content.right    = controller.right;

        controller_content.steering = controller.steering;

        controller_content.clutch   = controller.clutch;
        controller_content.brake    = controller.brake;
        controller_content.gas      = controller.gas;

        controllers_content.controller.push_back(controller_content);
      }
      output_->publish(controllers_content);
#endif
    }
  }
}
