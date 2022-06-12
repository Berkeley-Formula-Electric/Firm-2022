# Firm 2022

This repository stores all the firmware used on UC Berkeley's 2022 FSAE vehicle.


## System Overview

There are two CAN bus in the control system, which devices are all connected to the bus via RJ45 connectors.

We are using `T568A` type connection for RJ45.

The pinout of the connector is shown below

| CAN Signal | RJ45 Wire Index | RJ45 Color |
| ---------- | --------------- | ---------- |
| CAN1 High  | 1               | Dashed <span style="color:green">Green</span> |
| CAN1 Low   | 2               | <span style="color:green">Green</span> |
| CAN2 High  | 3               | Dashed <span style="color:orange">Orange</span> |
| CAN2 Low   | 4               | <span style="color:blue">Blue</span> |
| 12V        | 5               | Dashed <span style="color:blue">Blue</span> |
| GND        | 6               | <span style="color:orange">Orange</span> |
| 12V        | 7               | Dashed <span style="color:brown">Brown</span> |
| GND        | 8               | <span style="color:brown">Brown</span> |

**Note: 12V and GND are only connected from APPS to STEERINGWHEEL, used to supply power to STEERINGWHEEL. Thus, the STEERINGWHEEL must connect to APPS's RIGHT Ethernet connector.**


## Module Overview

### APPS

Reads accelerator pedal and brake pedal value, check for errorous readings, and send torque command to motor inverter.

### BMS

Constantly monitors battery cell voltages and temperatures, sends shut down command to the system if anything is wrong.

### DATALOGGER

Logs CAN bus data.

### STEERINGWHEEL

Sends `READY_TO_DRIVE` and `NOT_READY_TO_DRIVE` command to APPS.

### CANalyze

Used to debug CAN bus.


## FEB Library API

Run `scripts/update_libraries.py` after library update to sync the update to every project workspace.

Note that because STM32CubeIDE will NOT initialize peripherals that are not used in the project, so if you see compilation error in the library source code, you can simply delete that source file.

### Logger

```C
#include "FEB_logger.h"
```

#### void FEB_log(char *module, char *level, char *msg)

`module` is a string indicating the current module.

`level` is a string indicating critical level. Available options are 

- `CRITICAL`
- `ERROR`
- `WARNING`
- `INFO`
- `DEBUG`

`msg` is the actual string containing message.

##### Example

```C
if (HAL_CAN_Start(&hcan1) != HAL_OK) {
  while (1)
    FEB_log(MODULE_NAME, "CRITICAL", "CAN1 initialization error");
}

```

```C
char str[64];
sprintf(str, "buttons: %d %d\r\n", button_bank_0, button_bank_1);
FEB_log(MODULE_NAME, "DEBUG", str);

```

### CAN

```C
#include "FEB_CAN.h"
```

#### void FEB_CAN_initFilter(CAN_HandleTypeDef *CANx, uint32_t filter_id, uint32_t filter_mask)

`CANx` is either `&hcan1` or `&hcan2`.

`filter_id` and `filter_mask` together defines the filtering rule. With `filter_mask == 0` indicates all frames will be received by this node. 

#### HAL_StatusTypeDef FEB_CAN_transmit(CAN_HandleTypeDef *CANx, uint16_t can_id, uint8_t *data, uint16_t size, uint8_t is_blocking);

`CANx` is either `&hcan1` or `&hcan2`.

`can_id` is the id of the frame, in range of `(0, 0x7FF]`

`data` is a pointer pointing to the buffer containing message that is going to be transmitted.

`size` is the size of the frame, in range of `[0, 8]`.

`is_blocking` defines how the transmission will happen. When the bus is busy, if `is_blocking` is true, the function will keep waiting until the bus is free and transmit the message. If `is_blocking` is false, the function will drop this transmission attempt.


## CAN ID Allocation Table

| CAN ID | Device Name | Data Length | Description |
| ------ | ----------- | ----------- | ----------- |
| 0x0C0  | APPS        | 8           | APPS to RMS command data. First two bytes are uint16_t torque value, in 10x N·m. 5th index (starting from 0) is RMS enabled control bit. |
| 0x200  | Steering Wheel | 1        | Steering Wheel `ready to drive` state. 0 indicate ready to drive, non-zero value indicate not ready to drive. |
| 0x201  | APPS        | 8           | APPS pedal broadcast. Consists of two floats, [brake_pedal, acc_pedal] |
