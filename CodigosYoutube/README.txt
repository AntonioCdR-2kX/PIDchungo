Reaction Wheel Inverted Pendulum
Released 16.04.2022 together with Youtube video https://youtu.be/WObG2LoSEwQ
Created by Brick Experiment Channel

Run on terminal before use (Raspberry Pi Zero 2 W):
    pip install mpu9250-jmdev
    sudo pigpiod

Python dependencies:
    gpiozero (https://gpiozero.readthedocs.io/en/stable/)
    mpu9250-jmdev (https://pypi.org/project/mpu9250-jmdev/)

Tested with:
    Raspberry Pi Zero 2 W
    Elecrow SM9250MPU 9DOF IMU
    OKYSTAR TB6612FNG motor driver
    Lego EV3 Medium motor [45503]
    Lego PF Rechargeable Battery Box [8878]
    more Lego parts (look at the video)
    Raspberry OS 11 (Linux kernel 5.10.103)
    Thonny 3.3.14
    Python 3.9.2
    gpiozero 1.6.2
    mpu9250-jmdev 1.0.12

Files:
    inverted_pendulum.py  |  main implementation
    datalog.log           |  log from a long test run (shown in the video starting from 10:57)
    log_to_graph.py       |  create matplotlib graph using datalog.dat (requires matplotlib)
    log_to_video.py       |  create matplotlib graph video file using datalog.dat (requires matplotlib and ffmpeg)
    test_*.py             |  test individual parts
