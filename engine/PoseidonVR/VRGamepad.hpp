#pragma once

// CWR-VR: the VR controllers as a virtual SDL gamepad.
//
// The game already has full gamepad support with per-context bindings
// (infantry, vehicles, map, menus) that the player can rebind in the
// controls menu.  Feeding the controllers in as an Xbox-layout gamepad
// reuses all of it:
//   left stick -> left stick (rotated for head-relative walking)
//   triggers   -> triggers (right = fire, left = optics by default)
//   A B X Y    -> A B X Y
//   left stick click -> left stick click
//   grips      -> d-pad up / down (previous / next action)
//   menu       -> Escape (pause menu)
// The right stick is not forwarded: it turns the player and its click
// recentres (handled by the graphics backend).

struct SDL_Joystick;

namespace PoseidonVR
{

struct ControllerState;

class VRGamepad
{
public:
    bool Create();
    void Destroy();
    bool IsCreated() const { return _joystick != nullptr; }

    // moveYaw: radians the view heading is right of the body heading; the
    // left stick is rotated by it so "forward" means "where I look".
    void Update(const ControllerState& c, float moveYaw);

private:
    SDL_Joystick* _joystick = nullptr;
    unsigned int _id = 0;
    bool _menuWasDown = false;
    bool _escapeDown = false;
};

} // namespace PoseidonVR
