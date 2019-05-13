#ifndef DEFINITIONS
#define DEFINITIONS

#define _900degrees  15.70796326795
#define _450degrees  7.8598163397
#define RADtoDEGREES 57.29577951308

namespace gamepad {
  struct GamepadData{
    //buttons:
    bool A     = false;
    bool B     = false;
    bool X     = false;
    bool Y     = false;
    bool RSB   = false;
    bool LSB   = false;
    bool RB    = false;
    bool LB    = false;
    bool back  = false;
    bool start = false;
    bool guide = false;
    //directional pad:
    bool up    = false;
    bool down  = false;
    bool left  = false;
    bool right = false;
    //steering: (angle in radians)
    double steering = 0;
    //pedals: (0 -> 1 : 0 = not pressed - 1 = maximum pressure)
    double clutch   = 0;
    double brake    = 0;
    double gas      = 0;
  };

  enum class Message : unsigned {
    Normal    = 0,
    Warning   = 1,
    Attention = 2,
    Error     = 3
  };
}
#endif // DEFINITIONS

