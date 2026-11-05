# CWR-VR: builds and starts the game. Called by build.bat and play.bat.
# SPDX-License-Identifier: GPL-3.0-or-later
#
#   cwr.ps1 build            builds the game
#   cwr.ps1 play [args]      starts it in VR, building it first if needed
#   cwr.ps1 play --flat      starts it on the monitor
#
# The game data folder is asked for once and kept in vr.local.ini (git-ignored).

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$BuildDir = Join-Path $Root 'build\win-x64-clang-rwdi'
$GameExe = Join-Path $Root 'dist\x64-win-rwdi\PoseidonGame.exe'
$LocalIni = Join-Path $Root 'vr.local.ini'

function Stop-WithMessage([string]$Message)
{
    Write-Host ''
    Write-Host $Message -ForegroundColor Red
    exit 1
}

function Find-CMake
{
    $onPath = Get-Command cmake -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }

    # An earlier configure knows which one it used.
    $cache = Join-Path $BuildDir 'CMakeCache.txt'
    if (Test-Path $cache)
    {
        $line = Select-String -Path $cache -Pattern '^CMAKE_COMMAND:INTERNAL=(.+)$' | Select-Object -First 1
        if ($line -and (Test-Path $line.Matches[0].Groups[1].Value)) { return $line.Matches[0].Groups[1].Value }
    }

    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path $vswhere)
    {
        foreach ($install in & $vswhere -latest -prerelease -products * -property installationPath)
        {
            $inVS = Join-Path $install 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
            if (Test-Path $inVS) { return $inVS }
        }
    }

    $installed = Join-Path $env:ProgramFiles 'CMake\bin\cmake.exe'
    if (Test-Path $installed) { return $installed }

    Stop-WithMessage 'CMake was not found. Install it with Visual Studio (C++ CMake tools) or from cmake.org.'
}

function Invoke-Build
{
    $cmake = Find-CMake
    # CMakeUserPresets.json (git-ignored) holds this machine's tool paths.
    $preset = 'win-x64-clang-rwdi'
    $buildArgs = @('--build', $BuildDir)
    $userPresets = Join-Path $Root 'CMakeUserPresets.json'
    if ((Test-Path $userPresets) -and (Select-String -Path $userPresets -SimpleMatch "local-$preset" -Quiet))
    {
        $preset = "local-$preset"
        $buildArgs = @('--build', '--preset', $preset)
    }
    elseif (-not $env:VCPKG_ROOT)
    {
        Stop-WithMessage 'VCPKG_ROOT is not set. Install vcpkg and point VCPKG_ROOT at its folder.'
    }

    Push-Location $Root
    try
    {
        if (-not (Test-Path (Join-Path $BuildDir 'CMakeCache.txt')))
        {
            Write-Host "Configuring ($preset)..."
            & $cmake --preset $preset
            if ($LASTEXITCODE -ne 0) { Stop-WithMessage 'Configure failed, see the messages above.' }
        }
        Write-Host "Building ($preset)..."
        & $cmake @buildArgs --target PoseidonGame VRParts
        if ($LASTEXITCODE -ne 0) { Stop-WithMessage 'Build failed, see the messages above.' }
    }
    finally
    {
        Pop-Location
    }
    Write-Host "Done: $GameExe" -ForegroundColor Green
}

function Test-GameData([string]$Folder)
{
    return (Test-Path (Join-Path $Folder 'dta')) -or (Test-Path (Join-Path $Folder 'Addons'))
}

function Get-DataDir
{
    if (Test-Path $LocalIni)
    {
        $line = Select-String -Path $LocalIni -Pattern '^\s*data_dir\s*=\s*(.+?)\s*$' | Select-Object -First 1
        if ($line -and (Test-Path $line.Matches[0].Groups[1].Value)) { return $line.Matches[0].Groups[1].Value }
    }

    Write-Host 'Where is the game installed? Paste the folder from Steam (Manage > Browse local files).'
    $folder = (Read-Host 'Game folder').Trim().Trim('"')
    $remastered = Join-Path $folder 'Remastered'
    $dataDir = $folder
    if ($folder -and (Test-Path $remastered)) { $dataDir = $remastered }
    if (-not $folder -or -not (Test-GameData $dataDir))
    {
        Stop-WithMessage "No game data found in: $folder"
    }

    if (Test-Path $LocalIni)
    {
        Add-Content -Path $LocalIni -Value "data_dir=$dataDir" -Encoding UTF8
    }
    else
    {
        Set-Content -Path $LocalIni -Value "; Local machine settings for CWR-VR. Git-ignored; do not commit.", '[paths]', "data_dir=$dataDir" -Encoding UTF8
    }
    return $dataDir
}

function Start-Game([string[]]$GameArgs)
{
    if (-not (Test-Path $GameExe)) { Invoke-Build }
    $dataDir = Get-DataDir

    $flat = $GameArgs -contains '--flat'
    $list = @($GameArgs | Where-Object { $_ -and $_ -ne '--flat' })
    if (-not $flat) { $list = @('--vr') + $list }

    Write-Host "Starting the game from: $dataDir"
    if ($list.Count -gt 0)
    {
        Start-Process -FilePath $GameExe -WorkingDirectory $dataDir -ArgumentList $list
    }
    else
    {
        Start-Process -FilePath $GameExe -WorkingDirectory $dataDir
    }
}

$command = if ($args.Count -gt 0) { $args[0] } else { 'play' }
$rest = @($args | Select-Object -Skip 1)
switch ($command)
{
    'build' { Invoke-Build }
    'play' { Start-Game $rest }
    default { Stop-WithMessage "Unknown command: $command (use build or play)" }
}
