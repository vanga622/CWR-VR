@echo off
rem CWR-VR: builds the game.
rem SPDX-License-Identifier: GPL-3.0-or-later
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\cwr.ps1" build
pause
