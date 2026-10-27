# Repository Guidelines

## Project Structure & Module Organization
This directory contains STM32H723VGTx flight-controller firmware using STM32 HAL and FreeRTOS.
- `App/`: flight control, BMI088 acquisition/filtering/attitude, RC, ESC, telemetry, and blackbox; `App/storage/` contains SD storage support.
- `Core/`: CubeMX-generated initialization and interrupt handlers; `CtrlBoard-H7.ioc` is the hardware configuration.
- `Devices/WS2812/`: LED driver. `Drivers/`, `Middlewares/`, and `ThirdParty/` hold platform and dependency code.
- `MDK-ARM/`: Keil project, linker configuration, and build script.
- `tests/`: host C fixtures, hardware stubs, and Python regressions. `tools/` contains calibration/temperature analysis utilities. Root Markdown files document individual subsystems.

## Build, Test, and Development Commands
Run from this directory in PowerShell:
```powershell
./MDK-ARM/build_ws2812.ps1
./tests/pid_tuning/run.ps1
./tests/run_sd.ps1
python -B -m unittest discover -s tests -p 'test_bmi088_*analysis.py'
```
The build uses ARMCC 5.06 update 5 at `D:\keil5\ARM\ARMCC\Bin`, defaults to `-O3`, and produces AXF/HEX/map/log files under `MDK-ARM/ws2812-hal-verified/`. Alternatively, open `MDK-ARM/CtrlBoard-H7.uvprojx`, target `CtrlBoard-H7`, in Keil.

The PowerShell tests compile and run control/IMU and SD/blackbox regressions using MSVC; override `-VcVars` for your installation. They write some fixtures into sibling `../STM32F407VET6黑匣子/tests/build/`; SD round-trip tests also require that project's converter. Python tests require NumPy. Firmware execution requires the target board.

## Coding Style & Naming Conventions
Match surrounding code: application C generally uses four-space indentation, `snake_case.c/.h` filenames, module-prefixed functions such as `Bmi088_UpdateReadRate`, and uppercase macros. Preserve explicit unit suffixes such as `_rad_s` and `_m_s2`. Keep CubeMX edits inside `USER CODE` regions where applicable. Preserve existing file encoding. No repository formatter configuration was found; avoid unrelated reformatting.

## Testing Guidelines
C tests use assertions and HAL/RTOS stubs; Python tests use `unittest`. Name regressions `test_<feature>.c` or `test_<feature>.py` and register C cases in the appropriate runner. No numeric coverage threshold is configured. Test changed behavior and relevant failure paths; record baseline failures without weakening assertions. Host checks do not establish board timing or flight safety. Report unexecuted checks as “未执行验证”.

## Commit & Pull Request Guidelines
History uses short, descriptive Chinese summaries; no mandatory Conventional Commits format is evident. Keep commits focused. PRs should explain the affected subsystem, behavior change, validation commands/results, and hardware checks still needed. Link relevant issues and update subsystem documentation when interfaces change. Exclude generated binaries, `tests/build/`, and unrelated Keil user settings.
