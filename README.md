# Timing Synchronization Via Sensing

**ECE 535/635 - Networked Embedded Systems Design · UMass Amherst**<br>
**Instructor:** Fatima Anwar <br>
**Team:** Vytas Subacius, Phu Nguyen, Liam Earle, Sze Nga Wong <br>

We aim to develop a resource-efficient time synchronization protocol than existed timing services, hoping to contribute to the future world of network embedded systems with our experience in embedded systems, Linux, and machine learning (ML).

---

## 1. Motivation

In recent years, smart devices and connected technologies have become increasingly integrated into everyday life. From smart homes to autonomous vehicles, these technologies require network communication and time synchronization to operate reliably and efficiently. However, existing timing services can place a significant burden on devices with limited resources. Therefore, we want to leverage this time-stamped sensor data to synchronize the sensing devices.

**Our idea:** We will utilize ESP32 to collecting data from a sound and a motion sensor then Raspberry Pi will received and run the synchronization protocol.

## 2. Design Goals

| Goal | Description |
|---|---|
| **Processor Efficiency** | Real time processing on Raspberry Pi < 5%, ESP32 CPU Usage < 1% |
| **Precision** | +/- 1 ms |
| **Network Usage** | Low Bandwidth
| **Power Efficiency** | Low Power Usage


## 3. Deliverables

1. **Deliverable 1** - Characterize network delay between Raspberry Pi and the edge device
2. **Deliverable 2** - Estimate the relative clock drive between devices
3. **Deliverable 3** - Visualization of delay, offset, and drift relative to time 


## 4. System Blocks

![System Blocks](system_block.png)

## 5. Hardware / Software Requirements

### Hardware
| Item | Qty | Purpose |
|---|---|---|
| ESP32 | 2 | Collecting Data From Sensors |
| Raspberry Pi | 1 | Received and Run Synchronization Protocol |
| KY-037 | 1 | Sound Detection |
| HC-SR501 | 1 | Motion Detection|
| Breadboards & Jumper Cables | — | Connect Components |

### Software
- **Computer:** ArduinoIDE, C/C++, Python
- **Raspberry Pi:** Linux/Ubuntu, Python 
- **Collaboration:** Git/Github

## 6. Team Members and Responsibilities

Lead roles to assign: **Setup, Software, Networking, Writing, Research, Algorithm Design** (each member leads one or two; everyone contributes across the project).

| Member | Lead role(s) | Responsibilities |
|---|---|---|
| Sze Nga Wong | **Hardware Setup**, **Software** | Building sensing and timestamp hardware  |
| Liam Earle | **Networking**, **Software** | Implement BLE links between ESP32 and Pi and network characterization |
| Phu Nguyen | **Algorithm Design**, **Software** | Designing Synchronization Algorithm and implementation |
| Vytas Subacius | **Writing & Research**, **Software**| Research related work and implement software   |

## 7. Project Timeline

| Week | Dates | Milestone |
|---|---|---|
| 1 | Sep 28 – Oct 3 | Team formed, project selected, repo submitted on Canvas |
| 2 | Oct 5 - Oct 10| Hardware setup and development environments working |
| 3 | Oct 12 - Oct 17 | Wire the sensors to ESP32s, and get BLE link working between ESP32s and Raspberry Pi |
| 4 | Oct 19 - Oct 24 | Stream timestamped data from both ESP32s to the Raspberry Pi, set up logging and test sensor response time |
| 5 | Oct 26 - Oct 31 | Deliverable 1 - Measure network delay and jitter |
| 6 | Nov 2 - Nov 7 | Deliverable 2 - Long-run clock drift measurement and drift estimate |
| 7 | Nov 9 - Nov 14 | Detect sound and motion events on each ESP32 and pair up the detections of the same events |
| 8 | Nov 16 - Nov 21 | Synchronization algorithm running on the Raspberry Pi |
| 9 | Nov 23 - Nov 28 | Evaluate the +/- 1 ms goal and measure CPU usage on the ESP32 and Raspberry Pi|
| 10 | Nov 30 - Dec 5 | Deliverable 3 - Plots of delay, offset and drift over time |
| 11 | Dec 7 - Dec 12 | Final demo and report |

## 8. References

**Provided project references**
1. [Adeel Nasrullah; Fatima M Anwar], "[HAEST: harvesting Ambient Events to Synchronize Time across Heterogeneous loT Devices]," *[IEEE]*, [2024].
2. [Lex Fridman a; Daniel E. Brown a; William Angell a; Irman Abdić a; Bryan Reimer a, Hae Young Noh], "[Automated Synchronization of Driving Data Using Vibration and Steering Events]," *[ScienceDirect]*, [2016].
3. [Sandeep Singh Sandha; Joseph Noor; Fatima M. Anwar; Mani Srivastava], "[Exploiting Smartphone Peripherals for Precise Time Synchronization]," *[IEEE]*, [2019].
