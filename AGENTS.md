# AGENTS.md

## Build environment (Windows / PowerShell)

ESP-IDF **v5.4.1** is installed at `C:\Espressif\frameworks\esp-idf-v5.4.1`.

The activation script mis-detects the shell as `powershell.EXE` (case mismatch), so set
`ESP_IDF_SHELL` before sourcing `export.ps1`. Because each command runs in a fresh shell,
activate and run the IDF command in the **same** invocation:

```powershell
$env:ESP_IDF_SHELL="powershell.exe"; & "C:\Espressif\frameworks\esp-idf-v5.4.1\export.ps1" 2>$null | Out-Null; idf.py build
```

### Common commands

```powershell
# Configure for the target (already done; re-run if you delete sdkconfig)
idf.py set-target esp32

# Build
idf.py build

# Flash + open serial monitor
idf.py -p COMx flash monitor
```

## Project notes

- Target: **ESP32-WROOM** (dual-core, 4 MB flash, no PSRAM).
- Language: **C++** (`main/main.cpp`, later modules under `main/`).
- Hardware: GY-87 module, **MPU6050 only** over I2C.
  - **SDA = GPIO21**, **SCL = GPIO22**, 400 kHz.
- Do not commit `sdkconfig` or `build/` (see `.gitignore`).
