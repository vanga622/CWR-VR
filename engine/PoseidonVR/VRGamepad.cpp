#include <PoseidonVR/VRGamepad.hpp>
#include <PoseidonVR/VRSession.hpp>

#include <SDL3/SDL.h>

#include <cmath>

namespace PoseidonVR
{
namespace
{

Sint16 ToStick(float v)
{
    if (v > 1.0f)
        v = 1.0f;
    if (v < -1.0f)
        v = -1.0f;
    return static_cast<Sint16>(v * SDL_JOYSTICK_AXIS_MAX);
}

// SDL joystick drivers report triggers over the full axis range (released =
// -32768) and the gamepad layer maps that to 0..32767; a raw 0 would read as
// a half-pressed trigger, held forever.
Sint16 ToTrigger(float v)
{
    if (v < 0.0f)
        v = 0.0f;
    return ToStick(v * 2.0f - 1.0f);
}

void PushKey(SDL_Scancode sc, bool down)
{
    SDL_Event ev = {};
    ev.type = down ? SDL_EVENT_KEY_DOWN : SDL_EVENT_KEY_UP;
    ev.key.scancode = sc;
    ev.key.key = SDL_GetKeyFromScancode(sc, SDL_KMOD_NONE, false);
    ev.key.mod = SDL_KMOD_NONE;
    ev.key.down = down;
    SDL_PushEvent(&ev);
}

} // namespace

bool VRGamepad::Create()
{
    if (_joystick)
        return true;
    if (!SDL_InitSubSystem(SDL_INIT_GAMEPAD))
        return false;

    // All standard axes and buttons, so SDL maps index i to SDL_GAMEPAD_*
    // enum value i.
    SDL_VirtualJoystickDesc desc;
    SDL_INIT_INTERFACE(&desc);
    desc.type = SDL_JOYSTICK_TYPE_GAMEPAD;
    desc.naxes = SDL_GAMEPAD_AXIS_COUNT;
    desc.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
    desc.axis_mask = (1u << SDL_GAMEPAD_AXIS_COUNT) - 1;
    desc.button_mask = (1u << SDL_GAMEPAD_BUTTON_COUNT) - 1;
    desc.name = "CWR-VR controllers";

    const SDL_JoystickID id = SDL_AttachVirtualJoystick(&desc);
    if (!id)
        return false;
    _joystick = SDL_OpenJoystick(id);
    if (!_joystick)
    {
        SDL_DetachVirtualJoystick(id);
        return false;
    }
    _id = id;
    return true;
}

void VRGamepad::Destroy()
{
    if (!_joystick)
        return;
    if (_escapeDown)
        PushKey(SDL_SCANCODE_ESCAPE, false);
    SDL_CloseJoystick(_joystick);
    SDL_DetachVirtualJoystick(_id);
    _joystick = nullptr;
    _id = 0;
    _escapeDown = _menuWasDown = false;
}

void VRGamepad::Update(const ControllerState& c, float moveYaw, bool forwardB)
{
    if (!_joystick)
        return;

    // Stick in view space (x right, y forward) -> body space.
    const float cs = std::cos(moveYaw), sn = std::sin(moveYaw);
    const float forward = c.moveY * cs - c.moveX * sn;
    const float right = c.moveX * cs + c.moveY * sn;

    SDL_SetJoystickVirtualAxis(_joystick, SDL_GAMEPAD_AXIS_LEFTX, ToStick(right));
    SDL_SetJoystickVirtualAxis(_joystick, SDL_GAMEPAD_AXIS_LEFTY, ToStick(-forward)); // SDL: +Y is down
    SDL_SetJoystickVirtualAxis(_joystick, SDL_GAMEPAD_AXIS_LEFT_TRIGGER, ToTrigger(c.trigger[0]));
    SDL_SetJoystickVirtualAxis(_joystick, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, ToTrigger(c.trigger[1]));

    SDL_SetJoystickVirtualButton(_joystick, SDL_GAMEPAD_BUTTON_SOUTH, c.a);
    SDL_SetJoystickVirtualButton(_joystick, SDL_GAMEPAD_BUTTON_EAST, forwardB && c.b);
    SDL_SetJoystickVirtualButton(_joystick, SDL_GAMEPAD_BUTTON_WEST, c.x);
    SDL_SetJoystickVirtualButton(_joystick, SDL_GAMEPAD_BUTTON_NORTH, c.y);
    SDL_SetJoystickVirtualButton(_joystick, SDL_GAMEPAD_BUTTON_LEFT_STICK, c.thumbClick[0]);
    SDL_SetJoystickVirtualButton(_joystick, SDL_GAMEPAD_BUTTON_DPAD_UP, c.turnY > 0.7f);
    SDL_SetJoystickVirtualButton(_joystick, SDL_GAMEPAD_BUTTON_DPAD_DOWN, c.turnY < -0.7f);

    // Menu: Escape, held for one frame so the engine's edge detection sees
    // a press and a release.
    if (_escapeDown)
    {
        PushKey(SDL_SCANCODE_ESCAPE, false);
        _escapeDown = false;
    }
    if (c.menu && !_menuWasDown)
    {
        PushKey(SDL_SCANCODE_ESCAPE, true);
        _escapeDown = true;
    }
    _menuWasDown = c.menu;
}

} // namespace PoseidonVR
