@echo off
rem CWR-VR: starts the game in VR. play.bat --flat starts it on the monitor.
rem SPDX-License-Identifier: GPL-3.0-or-later
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\cwr.ps1" play %*
if errorlevel 1 pause
