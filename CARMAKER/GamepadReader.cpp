#include "GamepadReader.h"

GamepadReader::GamepadReader() :
  joysticks_(0, NULL),
  map_id_(0),
  gamepads_(0)
{
  if(SDL_Init(SDL_INIT_GAMECONTROLLER) != 0){
    std::cout << "Unable to initialize SDL: " << SDL_GetError() << std::endl;
    return;
  }

  GamepadData control;
  map_id_ = SDL_GameControllerAddMappingsFromFile("control_maps/logitech_g920_map.txt");

  if(SDL_NumJoysticks() > 0){
    for(int i = 0; i < SDL_NumJoysticks(); i++)
      if(SDL_IsGameController(i)){
        std::cout << "Index \'" << i << "\' is a compatible controller, named \'"
                  << SDL_JoystickNameForIndex(i) << "\'" << std::endl;
        SDL_Joystick *joy = SDL_JoystickOpen(i);

        if(joy){
          joysticks_.push_back(joy);
          gamepads_.push_back(control);
          std::cout << "Found a valid controller, named: "
                    << SDL_JoystickName(controller) << std::endl;
        }else
          std::cout << "Could not open gamecontroller " << i << ": "
                    << SDL_GetError() << std::endl;
      }
  }else{
    std::cout << "There are no controllers detected." << std::endl;
  }
}

GamepadReader::~GamepadReader(){
  for(uint i = 0; i < joysticks_.size(); i++)
    if(joysticks_[i])
      SDL_JoystickClose(joysticks_[i]);

  SDL_Quit();
}

const std::vector<GamepadData> *const GamepadReader::get_gamepad_data(){
  return &gamepads_;
}

const GamepadData *const GamepadReader::get_gamepad_data(const int index){
  assert(index >= 0 && gamepads_.size() > index);
  return &gamepads_[index];
}

void GamepadReader::read_gamepad(const int index){
  if(joysticks_.size() > 0){
    SDL_JoystickUpdate();

    uint8_t hat;
    int16_t axis;

    if(index >= 0 && joysticks_.size() > index){
      hat = SDL_JoystickGetHat(joysticks_[index], 0);

      gamepads_[index].up = false;
      gamepads_[index].down = false;
      gamepads_[index].left = false;
      gamepads_[index].right = false;

      switch(hat){
      case SDL_HAT_UP:
        gamepads_[index].up = true;
      break;
      case SDL_HAT_DOWN:
        gamepads_[index].down = true;
      break;
      case SDL_HAT_LEFT:
        gamepads_[index].left = true;
      break;
      case SDL_HAT_RIGHT:
        gamepads_[index].right = true;
      break;
      case SDL_HAT_RIGHTUP:
        gamepads_[index].right = true;
        gamepads_[index].up = true;
      break;
      case SDL_HAT_RIGHTDOWN:
        gamepads_[index].right = true;
        gamepads_[index].down = true;
      break;
      case SDL_HAT_LEFTUP:
        gamepads_[index].left = true;
        gamepads_[index].up = true;
      break;
      case SDL_HAT_LEFTDOWN:
        gamepads_[index].left = true;
        gamepads_[index].down = true;
      break;
      }

      //brake
      axis = SDL_JoystickGetAxis(joysticks_[index], 2);
      gamepads_[index].brake = 1.0 - (double)(axis + 32768) / 65535.0;

      //gas
      axis = SDL_JoystickGetAxis(joysticks_[index], 1);
      gamepads_[index].gas = 1.0 - (double)(axis + 32768) / 65535.0;

      //clutch
      axis = SDL_JoystickGetAxis(joysticks_[index], 3);
      gamepads_[index].clutch = 1.0 - (double)(axis + 32768) / 65535.0;

      //steering
      axis = SDL_JoystickGetAxis(joysticks_[index], 0);
      gamepads_[index].steering = (double)(axis + 32768) / 65535.0 * _900degrees - _450degrees;

      if(gamepads_[index].clutch == 0.5
         && gamepads_[index].brake == 1.0
         && gamepads_[index].gas == 1.0){
        gamepads_[index].clutch = 0.0;
        gamepads_[index].brake = 0.0;
        gamepads_[index].gas = 0.0;
      }

      //buttons:
      gamepads_[index].A     = (bool)SDL_JoystickGetButton(joysticks_[index], 0);
      gamepads_[index].B     = (bool)SDL_JoystickGetButton(joysticks_[index], 1);
      gamepads_[index].X     = (bool)SDL_JoystickGetButton(joysticks_[index], 2);
      gamepads_[index].Y     = (bool)SDL_JoystickGetButton(joysticks_[index], 3);
      gamepads_[index].RSB   = (bool)SDL_JoystickGetButton(joysticks_[index], 8);
      gamepads_[index].LSB   = (bool)SDL_JoystickGetButton(joysticks_[index], 9);
      gamepads_[index].RB    = (bool)SDL_JoystickGetButton(joysticks_[index], 4);
      gamepads_[index].LB    = (bool)SDL_JoystickGetButton(joysticks_[index], 5);
      gamepads_[index].back  = (bool)SDL_JoystickGetButton(joysticks_[index], 7);
      gamepads_[index].start = (bool)SDL_JoystickGetButton(joysticks_[index], 6);
      gamepads_[index].guide = (bool)SDL_JoystickGetButton(joysticks_[index], 10);

    }else if(index < 0){

      for(unsigned int i = 0; i < joysticks_.size(); i++){
        //directional PAD:
        hat = SDL_JoystickGetHat(joysticks_[i], 0);

        gamepads_[i].up = false;
        gamepads_[i].down = false;
        gamepads_[i].left = false;
        gamepads_[i].right = false;

        switch(hat){
        case SDL_HAT_UP:
          gamepads_[i].up = true;
        break;
        case SDL_HAT_DOWN:
          gamepads_[i].down = true;
        break;
        case SDL_HAT_LEFT:
          gamepads_[i].left = true;
        break;
        case SDL_HAT_RIGHT:
          gamepads_[i].right = true;
        break;
        case SDL_HAT_RIGHTUP:
          gamepads_[i].right = true;
          gamepads_[i].up = true;
        break;
        case SDL_HAT_RIGHTDOWN:
          gamepads_[i].right = true;
          gamepads_[i].down = true;
        break;
        case SDL_HAT_LEFTUP:
          gamepads_[i].left = true;
          gamepads_[i].up = true;
        break;
        case SDL_HAT_LEFTDOWN:
          gamepads_[i].left = true;
          gamepads_[i].down = true;
        break;
        }

        //brake
        axis = SDL_JoystickGetAxis(joysticks_[i], 2);
        gamepads_[i].brake = 1.0 - (double)(axis + 32768) / 65535.0;

        //gas
        axis = SDL_JoystickGetAxis(joysticks_[i], 1);
        gamepads_[i].gas = 1.0 - (double)(axis + 32768) / 65535.0;

        //clutch
        axis = SDL_JoystickGetAxis(joysticks_[i], 3);
        gamepads_[i].clutch = 1.0 - (double)(axis + 32768) / 65535.0;

        //steering
        axis = SDL_JoystickGetAxis(joysticks_[i], 0);
        gamepads_[i].steering = (double)(axis + 32768) / 65535.0 * _900degrees - _450degrees;

        if(gamepads_[i].clutch == 0.5 && gamepads_[i].brake == 1.0 && gamepads_[i].gas == 1.0){
          gamepads_[i].clutch = 0.0;
          gamepads_[i].brake = 0.0;
          gamepads_[i].gas = 0.0;
        }

        //buttons:
        gamepads_[i].A     = (bool)SDL_JoystickGetButton(joysticks_[i], 0);
        gamepads_[i].B     = (bool)SDL_JoystickGetButton(joysticks_[i], 1);
        gamepads_[i].X     = (bool)SDL_JoystickGetButton(joysticks_[i], 2);
        gamepads_[i].Y     = (bool)SDL_JoystickGetButton(joysticks_[i], 3);
        gamepads_[i].RSB   = (bool)SDL_JoystickGetButton(joysticks_[i], 8);
        gamepads_[i].LSB   = (bool)SDL_JoystickGetButton(joysticks_[i], 9);
        gamepads_[i].RB    = (bool)SDL_JoystickGetButton(joysticks_[i], 4);
        gamepads_[i].LB    = (bool)SDL_JoystickGetButton(joysticks_[i], 5);
        gamepads_[i].back  = (bool)SDL_JoystickGetButton(joysticks_[i], 7);
        gamepads_[i].start = (bool)SDL_JoystickGetButton(joysticks_[i], 6);
        gamepads_[i].guide = (bool)SDL_JoystickGetButton(joysticks_[i], 10);
      }
    }
  }
}
