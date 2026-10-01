# Compilează (și opțional flash-uiește) firmware/rx_audio pentru Waveshare RP2350-PiZero.
#
#   powershell -File tools\build_rx_audio.ps1            # doar compilare
#   powershell -File tools\build_rx_audio.ps1 -Flash     # compilare + flash (portul plăcii găsit singur)
#
# De ce un script: microfonul USB (usb_audio.cpp) cere stiva „Adafruit TinyUSB" și flag-urile CFG_TUD_AUDIO_*,
# care nu se pot pune din sketch (biblioteca TinyUSB se compilează separat și trebuie să le vadă).
param([switch]$Flash, [string]$Cli = "D:\Arduino\ArduinoCompiler\arduino-cli\arduino-cli.exe")

$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
$out = Join-Path $root "build\waveshare_rp2350_pizero\rx_audio"
$fqbn = "rp2040:rp2040:waveshare_rp2350_pizero:usbstack=tinyusb"
$flags = @(
  "-DCFG_TUD_AUDIO=1"
  "-DCFG_TUD_AUDIO_FUNC_1_DESC_LEN=TUD_AUDIO_MIC_ONE_CH_DESC_LEN"
  "-DCFG_TUD_AUDIO_FUNC_1_N_AS_INT=1"
  "-DCFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ=64"
  "-DCFG_TUD_AUDIO_ENABLE_EP_IN=1"
  "-DCFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX=98"        # 49 eșantioane × 2 octeți (48 ± 1 pe cadru)
  "-DCFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ=784"    # ~8 ms
) -join " "

& $Cli compile --fqbn $fqbn --build-property "build.extra_flags=$flags" --output-dir $out (Join-Path $root "firmware\rx_audio")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if (-not $Flash) { exit 0 }

# reset în BOOTSEL prin atingerea portului la 1200 baud, apoi picotool
$port = Get-CimInstance Win32_SerialPort | Where-Object { $_.PNPDeviceID -match "VID_2E8A" } | Select-Object -First 1 -ExpandProperty DeviceID
if ($port) {
  Write-Host "reset prin $port"
  $p = New-Object System.IO.Ports.SerialPort $port, 1200
  try { $p.Open(); $p.Close() } catch { }   # placa se deconectează chiar în timpul Open: normal
  Start-Sleep 4
} else { Write-Host "niciun port: presupun că placa e deja în BOOTSEL" }
$picotool = Get-ChildItem "$env:LOCALAPPDATA\Arduino15\packages\rp2040\tools\pqt-picotool" -Recurse -Filter picotool.exe | Select-Object -First 1 -ExpandProperty FullName
& $picotool load -x (Join-Path $out "rx_audio.ino.uf2")
