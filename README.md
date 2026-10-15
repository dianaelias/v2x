# V2X Latency & Trajectory Planning Stability Testbed

An end-to-end simulation testbed to evaluate the impact of V2X communication disturbances (latency, jitter, packet loss, and Age of Information) on the trajectory planning stability of Connected and Autonomous Vehicles (CAVs).

The framework bridges real-world V2X dataset trajectories (UrbanIng-V2X) with standardized ETSI Cooperative Awareness Messages (CAM) and a reactive Frenet trajectory planner operating within CommonRoad.

---

## System Architecture



---

## Key Features

- **ETSI CAM Generation:** Synthesizes standard-compliant Cooperative Awareness Messages (Release 2) from raw trajectory sequences (including , , , and ).
- **Configurable Network Disturbances:** Injects synthetic network delays (50 ms - 500 ms), jitter, and packet loss into a Python message queue.
- **Local Dynamic Map (LDM):** Manages incoming asynchronous V2X updates, tracks *Age of Information* (AoI), and provides an object interface for planning.
- **CommonRoad Integration:** Evaluates reactive Frenet planning decisions (path oscillations, jerk, Time-To-Collision) in Docker-isolated environments.

---

## Repository Structure



---

## Quick Start

### Prerequisites
- Docker & Docker Compose installed
- Python 3.11+ (if running locally without Docker)

### Execution via Docker

1. **Clone the repository:**
   

2. **Run the simulation pipeline:**
   

---

## Evaluation Metrics

The testbed measures the impact of V2X synthetic delays using:
- **Temporal Consistency:** Path deviations across consecutive re-planning steps.
- **Jerk (m/s³):** Derivative of longitudinal and lateral acceleration (passenger comfort).
- **Time-To-Collision (TTC):** Instantaneous collision risk at intersection conflict points.

---

## Master Thesis Context

This repository forms the experimental evaluation for the Master's Thesis:  
"Impact of V2X Communication Latency on Trajectory Planning Stability"  

- **Institution:** Department of Cooperative Systems, FernUniversitaet in Hagen  
- **Author:** Diana Elias  
