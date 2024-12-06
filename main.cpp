# Balancing Robot

A PID-controlled robot that stays upright with visual and audible feedback using Mbed OS.

## Features

- **Balance Control**: Keeps the robot balanced using a PID controller based on IMU data.
- **Visual Feedback**: Shows a happy face on a uLCD when balanced and a sad face when tilted.
- **Audible Alerts**: Plays sounds through a speaker when the robot tilts beyond a set threshold.

## Hardware Requirements

- **Microcontroller**: Compatible with Mbed OS 6.16.0
- **IMU Sensor**: ICM20948
- **Motors**: 2 DC motors with motor drivers
- **Display**: uLCD from MbedLibraryCollection
- **Speaker**: Connected to a PWM pin (p21)
- **Miscellaneous**: Jumper wires, breadboard, power supply

## Software Requirements

- **Mbed OS**: 6.16.0 (upgrade to 6.17.0 if possible)
- **Mbed CLI**: For building and managing the project
- **Compiler**: ARM Compiler 6 (ARMC6) recommended
- **Serial Terminal**: For debugging (e.g., PuTTY, Tera Term)

## Libraries Used

- **ICM20948 by Eric Leal**  
  Interfaces with the ICM20948 IMU sensor.  
  [GitHub Link](http://os.mbed.com/users/ericleal/code/ICM20948/)

- **Motor by Simon**  
  Controls DC motors.  
  [GitHub Link](http://os.mbed.com/users/simon/code/Motor/)

- **uLCD from MbedLibraryCollection by Daniel Bailey**  
  Manages the uLCD display functionalities.  
  [GitHub Link](https://github.com/danielcbailey/MbedLibraryCollection.git)

- **Mbed OS by ARM**  
  Core operating system for the project.  
  [GitHub Link](https://github.com/ARMmbed/mbed-os/)  
  *(Using commit `54e8693ef4ff7e025018094f290a1d5cf380941f` on version 6.16.0)*

## Setup

1. **Clone the Repository**
    ```bash
    git clone https://github.com/yourusername/balancing-robot.git
    cd balancing-robot
    ```

2. **Import Libraries**
    ```bash
    mbed add https://github.com/ericleal/ICM20948.git
    mbed add https://github.com/simon/Motor.git
    mbed add https://github.com/danielcbailey/MbedLibraryCollection.git
    ```

3. **Configure `mbed_app.json`**
    Ensure `mbed_app.json` is set to enable floating-point printing:
    ```json
    {
        "config": {
            "print_float": {
                "value": 1
            }
        },
        "target_overrides": {
            "*": {
                "platform.stdio-convert-newlines": true,
                "platform.stdio-baud-rate": 9600
            }
        }
    }
    ```

4. **Build and Flash**
    Replace `YOUR_TARGET` with your specific board (e.g., `NUCLEO_F411RE`):
    ```bash
    mbed compile -m YOUR_TARGET -t GCC_ARM --flash
    ```

## Usage

1. **Connect Hardware**
    - **IMU**: Pins `p28` (SDA) and `p27` (SCL)
    - **Motor 1**: Pins `p23`, `p6`, `p5`
    - **Motor 2**: Pins `p26`, `p7`, `p8`
    - **uLCD**: Pins `p9` (TX), `p10` (RX), `p11` (RESET)
    - **Speaker**: Pin `p21`

2. **Monitor Serial Output**
    Open your serial terminal at 9600 baud to see debug messages and PID outputs.

3. **Operate the Robot**
    Power on the robot. It should balance itself, display faces on the uLCD, and emit sounds when tilted.

## Tips

- **ICM20948 Setup**: Ensure secure connections to pins `p28` and `p27` to get accurate IMU readings.
- **Testing Motors**: Verify motor functionality with simple scripts before running the PID loop.
- **uLCD Compatibility**: Double-check that your uLCD model is supported by the library from MbedLibraryCollection.
- **PID Tuning**: Adjust `Kp`, `Ki`, and `Kd` in the code to fine-tune the balancing performance.

## License

MIT License
