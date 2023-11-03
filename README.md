# Rover
Radar chirp data collection platform based on the TI AWR1843/DCA1000EVM.


## Getting Started

Physical Hardware:
- [AWR1843Boost Evaluation Board](https://www.ti.com/tool/AWR1843BOOST) ($300); uses a 5v 3A power supply.
- [DCA1000EVM Capture Card](https://www.ti.com/tool/DCA1000EVM); powered via the AWR1843.
- 2 micro USB cables connected to the AWR1843 and to the `RADAR_FTDI` port on the DCA1000EVM.
    - **NOTE**: sketchy USB cables may cause the radar/capture card to fail to be detected.
- 1 Windows computer (GUI required).
- 1 Ubuntu 20.XX (focal) computer.

### Software

Radar (Windows) computer:
1. Install mmWave studio & dependencies. See the [DCA1000EVM Quick Start Guide](https://www.ti.com/tool/DCA1000EVM) for full instructions.
    - Make sure to set a static IP address (`192.168.30.33`) for the network interface used as shown in step 3.
    - Make sure to install the matlab runtime engine noted in step 4.
    - You may need to install the [mmWave SDK](https://www.ti.com/tool/MMWAVE-SDK).
    - Make sure that all 6 COM ports are detected as shown in Step 6 / Figure 4. If the `XDS110 Class` ports are not detected, see the note about installing [EMUPACK](http://processors.wiki.ti.com/index.php/XDS_Emulation_Software_Package).
2. Note which COM port is labeled "XDS110 Class Application/User UART". Todo: note where this goes.
3. Run (todo).
    - **NOTE**: you may need to disable windows firewall, which blocks the ports used by the radar by default.

LIDAR (Linux) computer:
1. Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Install dependencies for [ouster-ros](https://github.com/ouster-lidar/ouster-ros)
3. 
