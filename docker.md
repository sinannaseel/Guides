
### 📄 `docker_guide.md`

````markdown
# 🐳 Docker Guide for UR Robot + Simulation Demo

This guide documents how to use Docker to run the UR robot control system with CoppeliaSim simulation, using the `docker-compose.yml` setup provided.

---

## 📁 Project Structure

```bash
sas_tutorial_workspace/
└── docker/
    └── robot_demo/
        ├── compose.yml
        └── docker_guide.md  ← (this file)
````

---

## 🚀 Getting Started

### 1. ✅ Prerequisites

Make sure you have:

* [Docker](https://docs.docker.com/get-docker/) installed
* [Docker Compose](https://docs.docker.com/compose/install/) installed
* X11 forwarding enabled for GUI apps to work (CoppeliaSim)

Run:

```bash
xhost +local:root
```

---

## 🧠 Services Explained

### 🧪 `sas_ur_control_template` (Robot Controller)

* Built from GitHub:

  ```
  https://github.com/MarinhoLab/sas_ur_control_template.git
  ```
* ROS 2 launch file: `robot_and_simulation_example_py_launch.py`
* Runs setup script: `prepare_robot_demo.sh`
* Shares volume with simulator for file access

### 🎮 `coppeliasim` (Simulator)

* Runs the GUI simulation app `CoppeliaSim`
* Loads UR3e robot scene: `UR3e_480rev0.ttt`
* Uses host display and X11 for GUI rendering
* Shares volume with the control container

---

## 🛠️ How to Run the System

### Step 1: Start the Services

From `robot_demo/` directory:

```bash
docker compose up --build
```

Or in detached mode:

```bash
docker compose up -d --build
```

### Step 2: View the Simulation

CoppeliaSim should open on your host machine. You may need to allow GUI apps from Docker with:

```bash
xhost +local:docker
```

---

## 🧹 Stop & Clean Up

### Stop the services:

```bash
docker compose down
```

---

## 🖥️ Accessing the Containers (For Debugging)

### Get container names:

```bash
docker ps
```

Example output:

```
CONTAINER ID   IMAGE                                           NAMES
aaf13b0f2628   robot_and_simulation_demo-sas_ur_control_template   robot_and_simulation_demo-sas_ur_control_template-1
0107edf1b18a   murilomarinho/coppeliasim:noble_470rev4             robot_and_simulation_demo-coppeliasim-1
```

### Access a container's shell:

```bash
docker exec -it <container_name_or_id> /bin/bash
```

Example:

```bash
docker exec -it robot_and_simulation_demo-sas_ur_control_template-1 /bin/bash
```

If bash is not available, use:

```bash
docker exec -it <container> /bin/sh
```

---

## 📝 Editing Files Inside the Container

1. Access the container shell (see above).
2. Navigate to:

   ```bash
   cd /root/sas_tutorial_workspace/src/sas_ur_control_template
   ```
3. Use an editor like `vi`:

   ```bash
   vi some_script.py
   ```

If `vi` or `nano` are missing and you have `apt`, you can install:

```bash
apt update && apt install nano
```

---

## 🔁 Persisting Changes

By default, files inside containers are **not saved** unless:

### Option 1: Use Named Volume (Already Used)

* The Compose file uses a volume:

  ```yaml
  volumes:
    - sas-ur-control-template-repo-data:/root/sas_tutorial_workspace/src/sas_ur_control_template/
  ```

* This persists data across container restarts.

### Option 2: Use Local Folder Bind Mount (Recommended for Development)

Instead of using a named volume, mount a local folder:

```yaml
volumes:
  - ../sas_ur_control_template:/root/sas_tutorial_workspace/src/sas_ur_control_template/
```

Then you can edit files locally and see changes immediately inside the container.

---

## 🔧 Environment Variables

You can pass `ROS_DOMAIN_ID` from your host machine to isolate ROS 2 topics:

```bash
export ROS_DOMAIN_ID=10
docker compose up
```

---

## 🐳 Common Docker Commands

| Task                          | Command                                 |
| ----------------------------- | --------------------------------------- |
| Start services                | `docker compose up`                     |
| Start in background           | `docker compose up -d`                  |
| Stop services                 | `docker compose down`                   |
| See running containers        | `docker ps`                             |
| Shell into a container        | `docker exec -it <container> /bin/bash` |
| List images                   | `docker images`                         |
| Remove all stopped containers | `docker container prune`                |
| Remove all unused volumes     | `docker volume prune`                   |

---

## ✅ Notes & Tips

* Always restart after making changes to `compose.yml`:

  ```bash
  docker compose down
  docker compose up --build
  ```

* If GUI doesn't show:

  ```bash
  xhost +local:root
  ```

* Use `network_mode: host` only if you're running on **Linux**.

---

## 📦 Volumes Used

```yaml
volumes:
  sas-ur-control-template-repo-data:
```

Used to persist and share project files between the ROS container and the simulation.

---

## 📚 References

* [CoppeliaSim](https://www.coppeliarobotics.com/)
* [ROS 2 Documentation](https://docs.ros.org/en/foxy/index.html)
* [Docker Compose](https://docs.docker.com/compose/)

---

```

---

Let me know if you want to add sections like "Troubleshooting" or "Customizing the Simulation Scene" — or if you want this turned into a downloadable `.md` file!
```
