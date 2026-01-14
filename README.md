# Robotic Control Interface (URCI)

[![ROS Version](https://img.shields.io/badge/ROS-Noetic%2FHB-blue.svg)](http://wiki.ros.org/noetic)
[![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED.svg?logo=docker&logoColor=white)](https://www.docker.com/)
[![Status](https://img.shields.io/badge/Status-Active-success.svg)]()

## 📖 Overview
**Robotic Control Interface** is a distributed teleoperation system designed to decouple real-time control loops from high-level data visualization. 

By leveraging **Docker containers** across dual **Raspberry Pi** units, this project ensures that heavy visualization tasks do not introduce latency into the critical control path. It features a custom **PID controller** augmented with an **Extended Kalman Filter (EKF)** to reject sensor noise from analog joysticks (via ADS1115) and estimate the true state of the input commands for smooth TurtleBot navigation in Gazebo.

## 🚀 Key Features
* **Distributed Architecture:** Separates the Control Node (Real-time) from the Visualization Node using ROS communication.
* **Noise Rejection:** Implements an **Extended Kalman Filter (EKF)** to smooth noisy ADC signals from analog joysticks.
* **Containerization:** Fully containerized using **Docker** to ensure reproducibility and easy deployment on ARM64 architectures.
* **Hardware Integration:** Low-level I2C interfacing with ADS1115 ADC modules.
* **Simulation Ready:** Pre-configured to drive TurtleBot3 in Gazebo simulations.

## 🛠️ System Architecture

The system is composed of two main subsystems running in separate containers/devices:

1.  **Acquisition & Control Node (Raspberry Pi A):**
    * Reads raw voltage from Joystick via **ADS1115 (I2C)**.
    * **EKF Step:** Predicts state and corrects based on measurement.
    * **PID Step:** Computes the control effort to reach the target velocity.
    * Publishes `/cmd_vel` topics.

2.  **Visualization & Monitor Node (Raspberry Pi B / PC):**
    * Subscribes to raw data and filtered states.
    * Visualizes signal quality and control response (rqt_plot / Foxglove).

---

## 🧮 Control Theory: EKF Implementation

To handle the stochastic noise inherent in low-cost analog sensors, the system models the joystick input as a dynamic system.

**State Prediction:**
$$
\hat{x}_{k|k-1} = F_k \hat{x}_{k-1|k-1} + B_k u_k
$$

**Measurement Update:**
$$
K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
$$
$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H_k \hat{x}_{k|k-1})
$$

*Where $R_k$ represents the measurement noise covariance matrix derived from sensor characterization.*

---

## ⚡ Hardware Requirements
* 2x Raspberry Pi 4 (or 1 RPi + 1 PC for hybrid setup).
* 1x Analog Joystick module.
* 1x ADS1115 ADC Module (16-bit).
* Jumper wires & Breadboard.
