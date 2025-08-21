# DesmoSAT Firmware Monorepo

## Overview
- Boards: Main / BMS / Custom-A / Custom-B / Landing / GroundControlCenter
- Bus: CAN 2.0B(FDCAN compatible), Std ID, Big-Endian (예시)
- Build: STM32CubeIDE 1.xx / ARM GCC x.y.z

## Repo Layout
- /MainModule/STM32
- /BMS/STM32
- /CustomModule(A)/STM32
- /CustomModule(B)/STM32
- /LandingModule/STM32
- /GroundControlCenter/STM32
- /common (drivers, can_proto.h, log, utils)

## Build & Flash
```bash
# Headless build (example via make/cmake or CubeIDE headless)
# tools/ci/build_main.sh
