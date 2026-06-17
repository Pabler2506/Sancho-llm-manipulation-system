# Sancho: LLM-Assisted Mobile Manipulation System in ROS 2

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python)
![Gemini](https://img.shields.io/badge/Google_Gemini-API-4285F4?style=for-the-badge&logo=google)

This repository contains the software architecture for an autonomous home assistant robot ("Sancho"). The core of this project bridges the gap between high-level natural language reasoning and low-level robotic execution by integrating **Large Language Models (LLMs)** with asynchronous **Behavior Trees** and the **Nav2** autonomous navigation stack.

Developed as a Final Degree Project (Trabajo Fin de Grado) in Electronics, Robotics, and Mechatronics Engineering at the University of Málaga (UMA).

##  Key Features

* **Semantic Reasoning:** Uses Google Gemini (Flash Lite) to interpret ambiguous natural language voice commands and translate them into structured JSON task sequences.
* **Asynchronous Control:** Implements a highly modular Behavior Tree using `py_trees`, executing control loops at 10 Hz without blocking the main ROS 2 thread during API calls or navigation timeouts.
* **Fault Tolerance:** Features built-in recovery behaviors (e.g., ±90º neck scans if an object is not detected) and automatic safe-return fallbacks if a target becomes unreachable.
* **Simulation-Ready:** Fully configured to interface with CoppeliaSim environments via ROS 2 topics and services.

---

##  System Architecture & Execution Flow

The architecture is divided into two mutually exclusive threads to guarantee real-time reactivity: an asynchronous listening loop (0.2 Hz) and the main Behavior Tree control loop (10 Hz). They communicate seamlessly through a shared global memory (Blackboard).

