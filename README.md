# Sancho: LLM-Assisted Mobile Manipulation System in ROS 2

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python)
![Gemini](https://img.shields.io/badge/Google_Gemini-API-4285F4?style=for-the-badge&logo=google)

***
*** Warning: this repositorie is still a work in progress, expect things not to work yet. ***
***

This repository contains the software architecture for a **simulated** autonomous home assistant robot ("Sancho"). The core of this project bridges the gap between high-level natural language reasoning and low-level robotic execution by integrating **Large Language Models (LLMs)** with asynchronous **Behavior Trees** and the **Nav2** autonomous navigation stack.

Developed as a Final Degree Project (Trabajo Fin de Grado) in Electronics, Robotics, and Mechatronics Engineering at the University of Málaga (UMA).

##  Key Features

* **Semantic Reasoning:** Uses Google Gemini (Flash Lite) to interpret ambiguous natural language voice commands and translate them into structured JSON task sequences.
* **Asynchronous Control:** Implements a highly modular Behavior Tree using `py_trees`, executing control loops at 10 Hz without blocking the main ROS 2 thread during API calls or navigation timeouts.
* **Fault Tolerance:** Features built-in recovery behaviors (e.g., ±90º neck scans if an object is not detected) and automatic safe-return fallbacks if a target becomes unreachable.
* **Simulation-Ready:** Fully configured to interface with CoppeliaSim environments via ROS 2 topics and services.

---

##  System Architecture & Execution Flow

The architecture is divided into two mutually exclusive threads to guarantee real-time reactivity: an asynchronous listening loop (0.2 Hz) and the main Behavior Tree control loop (10 Hz). They communicate seamlessly through a shared global memory (Blackboard).

![Flow Chart of the Arquitecture](DiagramaFlujoArquitectura.png)

##  Prerequisites
To run this project, you will need the following environment:
* OS: Ubuntu 24.04 (Native or via WSL2)
* Middleware: ROS 2 Jazzy Jalisco
* Simulator: CoppeliaSim Edu
* API Keys: A valid Google Gemini API Key.
* Python Dependencies: pip install py_trees py_trees_ros google-genai SpeechRecognition
  _(Note: If using Ubuntu 24.04, install system-wide packages via apt or use --break-system-packages at your own risk)._

---

##  Installation & Build
Clone this repository into your local machine:
```
git clone [https://github.com/YourUsername/your-repo-name.git](https://github.com/YourUsername/your-repo-name.git)
cd your-repo-name
```
Build the workspace using colcon:
```
colcon build --symlink-install
```
Source the environment:
```
source install/setup.bash
```

##  Execution
Export your Gemini API key to the environment variables:
```
export GEMINI_API_KEY="your_api_key_here"
```
Launch the CoppeliaSim environment and ensure the robotic model is running.
Launch the Nav2 stack with the custom map parameters.
```
```
Run the Image Proccessing node:
```
```
Run the main cognitive node:
```
ros2 run gemini_cerebro launch_brain
```
The robot will greet you and enter an active listening state, waiting for natural language instructions (e.g.: "Sancho, I'm thirsty, bring me something to drink from the kitchen").

## Author
Pablo Sierra Cano. _Degree in Electronics, Robotics, and Mechatronics Engineering._ Universidad de Málaga (UMA).

---

##  Descripción en Español:

Este repositorio contiene la arquitectura de software para un robot asistente del hogar autónomo **simulado** ("Sancho"). El núcleo de este proyecto acorta la brecha entre el razonamiento de alto nivel en lenguaje natural y la ejecución robótica de bajo nivel mediante la integración de **Modelos de Lenguaje Grande (LLMs)** con **Árboles de Comportamiento** asíncronos y el *stack* de navegación autónoma **Nav2**.

Desarrollado como Trabajo Fin de Grado en Ingeniería Electrónica, Robótica y Mecatrónica en la Universidad de Málaga (UMA).

##  Características Principales

* **Razonamiento Semántico:** Utiliza Google Gemini (Flash Lite) para interpretar comandos de voz ambiguos en lenguaje natural y traducirlos a secuencias de tareas estructuradas en formato JSON.
* **Control Asíncrono:** Implementa un Árbol de Comportamiento altamente modular utilizando `py_trees`, ejecutando bucles de control a 10 Hz sin bloquear el hilo principal de ROS 2 durante las llamadas a la API o los tiempos de espera (*timeouts*) de navegación.
* **Tolerancia a Fallos:** Cuenta con comportamientos de recuperación integrados (p. ej., barridos de cuello a ±90º si no se detecta un objeto) y rutinas automáticas de retorno seguro si un objetivo resulta inalcanzable.
* **Preparado para Simulación:** Totalmente configurado para interactuar con entornos de CoppeliaSim a través de tópicos y servicios de ROS 2.

---

##  Arquitectura del Sistema y Flujo de Ejecución

La arquitectura se divide en dos hilos mutuamente excluyentes para garantizar la reactividad en tiempo real: un bucle de escucha asíncrono (0.2 Hz) y el bucle de control principal del Árbol de Comportamiento (10 Hz). Ambos se comunican de forma fluida a través de una memoria global compartida (Blackboard).

##  Prerequisitos
Para ejecutar este proyecto, necesitarás las siguientes dependencias:
* SO: Ubuntu 24.04 (Nativo o vía WSL2)
* Middleware: ROS 2 Jazzy Jalisco
* Simulador: CoppeliaSim Edu
* API Keys: Una API Key válida de Google Gemini.
* Dependencias de Python: pip install py_trees py_trees_ros google-genai SpeechRecognition
  _(Nota: Si utilizas Ubuntu 24.04, instala los paquetes paquetes de todo el sistema a través de apt o utiliza la opción --break-system-packages bajo tu propia responsabilidad._

---

##  Instalación y compilación
Clona este repositorio en tu ordenador local:
```
git clone [https://github.com/YourUsername/your-repo-name.git](https://github.com/YourUsername/your-repo-name.git)
cd your-repo-name
```
Compila el espacio de trabajo usando colcon:
```
colcon build --symlink-install
```
Haz un source del entorno:
```
source install/setup.bash
```

##  Ejecución
Exporta tu Gemini API key a las variables de entorno:
```
export GEMINI_API_KEY="your_api_key_here"
```
Inicia el entorno CoppeliaSim y comprueba que el modelo esté en funcionamiento.
Inicia la pila Nav2 con los parámetros del mapa personalizados.
```
```
Ejecuta el nodo de procesamiento de imágenes:
```
```
Ejecuta el nodo cognitivo principal:
```
ros2 run gemini_cerebro launch_brain
```
El robot te dará la bienvenida y entrará en un estado de escucha activa, a la espera de instrucciones en lenguaje natural (por ejemplo: «Sancho, tengo sed, tráeme algo de beber de la cocina»).

## Autor
Pablo Sierra Cano. _Grado en Ingeniería Electrónica, Robótica y Mecatrónica._ Universidad de Málaga (UMA).
