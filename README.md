# A HYBRID COMPRESSED SENSING APPROACH FOR IMAGE TRANSMISSION IN LORA-BASED IOT SYSTEMS

This project demonstrates the use of an **ESP32-CAM** with **compressed sensing** techniques to capture image data, transmit it via **LoRa** using the **CubeCell AB01**, and reconstruct the image on a PC using **Python**.

## Overview
This project focuses on efficient image data transmission by utilizing compressed sensing for reducing data size. The **ESP32-CAM** captures images and performs compressed sensing to encode the data. The encoded data is then transmitted over **LoRa** using the **CubeCell AB01** module. On the receiving end, the data is reconstructed into an image using a **Python** script on the PC.

### Key Features:
- **Compressed Sensing** for efficient image compression and transmission
- **LoRa communication** using CubeCell AB01 for long-range data transmission
- **ESP32-CAM** for capturing images
- **Python-based reconstruction** of the compressed image

## Hardware Components
- **ESP32-CAM**: A small camera module with an integrated ESP32 microcontroller used for image capturing and initial data processing.
- **CubeCell AB01**: A LoRa-based microcontroller board for transmitting data over long distances.
- **LoRa Antenna**: For improving communication range.
- **PC**: To receive and reconstruct the transmitted image using Python.

## Software Components
- **Arduino IDE**: For programming the ESP32-CAM.
- **Python**: For image reconstruction on the PC.
- **PyWavelets**: A Python library for performing wavelet transforms for image reconstruction.
- **Matplotlib**: Used for visualizing the reconstructed image.

## Files and installation guide

**ESP32-CAM Files**
- [Main ESP32-CAM code and support files]()

**CubeCell-AB01 Files**
- [LoRa transmitter Code for AB01](https://github.com/Zydrax740/Compressed-Sensing-on-IOT-Devices/blob/main/CubeCell_AB01_Transmitter.ino)
- [LoRa Receiver Code for AB01](https://github.com/Zydrax740/Compressed-Sensing-on-IOT-Devices/blob/main/CubeCell_AB01_Transmitter.ino)

**Python Code for Reconstruction on PC**
- [Reconstruction Code](https://github.com/Zydrax740/Compressed-Sensing-on-IOT-Devices/blob/main/ReconstructionCode.py)


### 1. Install Dependencies for ESP32-CAM:
- Install the ESP32 Support files in the Arduino IDE or PlatformIO.

### 2. Install Dependencies for CubeCell-AB01:
- [Download USB Serial connection driver](https://docs.heltec.org/general/establish_serial_connection.html#establish-serial-connection)
- Install the CubeCell AB01 Support files for LoRa communication in the Arduino IDE or Platform IDE

### 3. Reconstruction Code on PC:
- [Download VisualCode](https://code.visualstudio.com/download)
- [Install PyWavelets](https://pywavelets.readthedocs.io/en/latest/install.html)
- [Install Matplotlib](https://matplotlib.org/stable/install/index.html)
  

