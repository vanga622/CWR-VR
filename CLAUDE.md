# Project brief for Claude Code

This repository is a fork of the Arma: Cold War Assault Remastered engine (codename Poseidon, from github.com/BohemiaInteractive/CWR) that adds native VR through OpenXR.

## Goal

Full room-scale VR for infantry play: stereo rendering with head tracking, motion controllers, the weapon held and aimed with the hand, physical manual reloading. Target hardware: Meta Quest / Pico over Virtual Desktop (VDXR runtime), SteamVR as fallback.

## Rules

- License: GPL-3.0-or-later. Keep all license headers. New files get the same license.
- Never use the names "Arma" or "Operation Flashpoint" for this fork, its binary, window title or logo. Working name: CWR-VR (change if the owner picks another).
- Never commit or package game data (PBOs, models, textures, sounds, missions).
- Flat (non-VR) play must keep working. Every VR change sits behind `--vr` or a `vr.ini` switch.
- Small commits, one working step each. Build and run before committing.

## Build and run

```
cmake --preset win-x64-clang-rwdi
cmake --build build/win-x64-clang-rwdi
```
Or `build.bat` and `play.bat` (`scripts/cwr.ps1`): they find CMake, use the local preset from `CMakeUserPresets.json`, and start the game from `data_dir`. `play.bat --flat` runs without VR.

Game data: the owner's Steam install of Arma: Cold War Assault Remastered (v3.05). Ask the owner for the path once and store it in `vr.local.ini` (git-ignored).

## Where things live

- Frame loop: `engine/Poseidon/World/World.cpp` (scene passes, then `GEngine->FinishDraw()` / `NextFrame()`).
- Present: `engine/PoseidonGL33/EngineGL33_VertexBuffer.cpp`, `EngineGL33::BackToFront()`.
- Offscreen target: `BindFrameRenderTarget()`, SSAA FBO in `engine/PoseidonGL33/EngineGL33.hpp`.
- Camera and projection: `engine/Poseidon/World/Scene/Camera/Camera.*` (projection is symmetric today; VR needs asymmetric).
- Camera modes: `World.hpp` (`CamInternal` = first person).
- Input: `engine/Poseidon/Input/` (`UserAction`, `InputSubsystem`, `InfantryController`).
- Aim and fire: `SoldierOld::GetWeaponDirection`, `FireWeapon` in `engine/Poseidon/World/Entities/Infantry/`.
- Reload: `UAReloadMagazine` handling in `SoldierOldActions.cpp`, `ReloadMagazine`, `MFReload`.
- Test harness: `engine/Trident` (`tri`), game flag `--harness <port>`, screenshot capture.
- Debug UI: ImGui `DebugOverlay`.

Put new VR code in `engine/PoseidonVR/` as its own library, and keep edits to existing engine files small and marked with `// VR:`.

## Working method

- The owner tests in the headset and reports in plain words. Turn each report into one concrete fix.
- Everything VR logs to `vr.log` (runtime name, extensions, swapchain formats, frame timing, errors). Read it after every run.
- Tunable values (hand offsets, holster positions, IPD scale) live in `vr.ini`, re-read at runtime, with ImGui sliders.
- Add a fake-headset mode (`--vr-fake`) with scripted head/hand poses and a `--vr-mirror` side-by-side eye capture, so stereo and weapon logic can be checked from screenshots without a headset.

## Milestones (current one marked)

- [x] M0 Build and run the unmodified game
- [x] M1 OpenXR session, flat game on a virtual screen, log runtime extensions (check `XR_KHR_opengl_enable` on VDXR; fallback SteamVR or `WGL_NV_DX_interop2` into a D3D11 swapchain)
- [x] M2 Stereo rendering, head tracking, asymmetric FOV, both-eye culling, HUD on a quad layer (culling is still per eye; a shared cull is a later optimisation)
- [x] M3 OpenXR actions mapped to `UserAction`s, turning, recentre (controllers reach the game as a virtual SDL gamepad, so the game's gamepad bindings apply)
- [x] M4 Weapon at the controller pose, fire from the barrel, two-handed grip
- [x] M5 Arm IK, eye height follows stance (stand/crouch/prone) (rigid arm follow, not true IK; the game has no rifle crouch, so a real crouch only lowers the view)
- [ ] M6 Manual reload: eject, grab magazine from pouch, insert, charge  ← current
- [ ] M7 Vehicles, binoculars/scopes, map in hand
