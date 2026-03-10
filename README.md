# Franka Prisma Master Thesis

> ⚠️ **Notice / Avviso**
> **This project is currently under active development as part of a Master's Thesis work.**
> *Questo progetto è un lavoro di tesi magistrale ed è attualmente in via di sviluppo.*

## Overview

This repository contains the ROS 2 workspace (`mm_ws`) developed for my Master's Thesis. The project involves the simulation and control of Franka Research 3 manipulators within a Gazebo environment, focusing on perception and reasoning pipelines to perform complex robotic tasks.

## Repository Structure

- `src/`: ROS 2 packages containing the core logic (perception, reasoning, and control).
- `docs/`: Project documentation, architectural diagrams, and pipeline walkthroughs.
- `Dockerfile` & `docker-compose.yml`: Docker configuration files to run the project in an isolated and reproducible environment.
- `scripts/`: Utility scripts for environment setup and execution.

## Getting Started

This project is set up to run using Docker to ensure dependency encapsulation. 

```bash
# Example command to start the environment (WIP)
docker-compose up
```

*(Detailed instructions regarding setup, execution, and testing will be added upon the completion of the project).*
