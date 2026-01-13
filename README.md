# STM32F103 Blue Pill FreeRTOS Demo

## Overview
This project demonstrates the use of FreeRTOS on the STM32F103C8 (“Blue Pill”) microcontroller.  
It includes tasks, queues, semaphores, mutexes, and ISR → task communication.

---

## Hardware Setup
- **MCU:** STM32F103C8 (Blue Pill)  
- **LED:** PC13 (onboard)  
- **Button:** PA0 (external, EXTI)  
- **UART:** USART1 (PA9 TX, PA10 RX)  

---

## RTOS Architecture

| Task Name       | Priority       | Role |
|-----------------|---------------|------|
| ControlTask     | Normal        | Periodically generates events & sends data to `tickQueue` |
| LedTask         | Low           | Waits for `ledSemaphore`, toggles LED (`ledMutex` protected) |
| TickTask        | Low           | Receives tick count from `tickQueue`, prints via UART (`printMutex`) |
| ButtonTask      | Above Normal  | Waits for button ISR semaphore, logs event, optionally triggers LED |
| StatsTask       | Low           | Prints task statistics and CPU usage every 10s |

---

## RTOS Objects

| Object           | Type          | Purpose |
|-----------------|---------------|---------|
| `tickQueue`      | Queue         | Transfer tick count between tasks |
| `ledSemaphore`   | Binary Semaphore | Event sync between `ControlTask` / ISR and `LedTask` |
| `ledMutex`       | Mutex         | Protect LED GPIO from concurrent access |
| `printMutex`     | Mutex         | Protect UART `printf` from concurrent access |

---

## CubeMX Configuration

1. **FreeRTOS CMSIS-V2**
   - Enable Mutexes, Semaphores, Timers, and Trace Facility
   - Use **Dynamic Allocation (heap_4.c)** for learning
2. **GPIO**
   - LED → Output (PC13)
   - Button → Input with EXTI (PA0)
3. **UART1**
   - TX: PA9, RX: PA10, 2 Mbps
   - Redirect `printf` to UART
4. **Timer**
   - TIM13 for runtime stats / optional periodic events

---

## Concept Summary

- **Queue** → Safe data transfer between tasks  
- **Semaphore** → Signal events (task ↔ task / ISR ↔ task)  
- **Mutex** → Protect shared resources like UART & GPIO  
- **ISR** → Hardware-triggered events (Button EXTI)  
- **Tasks** → Periodic or event-driven code execution  

---

## Notes
- Dynamic allocation is used for learning; **static allocation recommended for automotive / safety-critical systems**  
- This project is ideal for beginners to intermediate FreeRTOS learners  
- Supports STM32F103C8 (Blue Pill) board  

---

