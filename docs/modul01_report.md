### Table 1.1

| No | Parameter | Unit | Result |
| :--- | :--- | :--- | :--- |
| 1 | Base image "docker pull" time | s | 157.534 |
| 2 | First image build time (without cache) | s | 111.504 |
| 3 | Second image build time (with cache) | s | 1.797 |
| 4 | Image size of "robotics-lab:dev" | MB | 1890 |
| 5 | Container startup time ("docker compose up") | s | 12.415 |
| 6 | First "colcon build" time | s | 1.588 |
| 7 | Number of packages built | count | 1 |
| 8 | Commit hash | - | 12e1e09 |


### Table 1.2

| Aspect | Bind mount "../src:/ws/src" | Named volume "ws_install" |
| :--- | :--- | :--- |
| Visible on host | Yes | No |
| Persists after "docker compose down" | Yes | Yes |
| Suitable for source code | Yes | No |
| Suitable for build artifacts | No | Yes |


## Document Troubleshooting (Sections 1.10 & 1.14)

| Problem | Symptom | Root Cause | Solution | Verification |
| :--- | :--- | :--- | :--- | :--- |
| Missing GUI display permissions | `Command 'xhost' not found` when setting permissions on host, and `rqt_graph` failing to open. | The `x11-xserver-utils` package was missing on the Ubuntu host, preventing the container from forwarding its display to the host server. | Run `sudo apt install x11-xserver-utils` on the host, then run `xhost +local:docker`. | The `rqt_graph` and `rviz2` GUI windows successfully open on the screen. |
| Build artifact permission denied | `PermissionError: [Errno 13] Permission denied: 'build/.built_by'` when running colcon build. | Docker automatically created the named volumes (`ws_build`, `ws_install`) as the `root` user, preventing the non-root `dev` user from writing to them. | Execute `docker compose exec -u root dev chown -R dev:dev /ws` from the host to fix directory ownership. | `colcon build --symlink-install` completes without permission errors. |
| Unrecognized ROS 2 command | `-bash: ros2: command not found` when running `ros2 pkg create`. | The command was executed on the host machine (Ubuntu WSL), which does not have ROS 2 installed, instead of inside the Docker container. | Run `docker compose exec dev bash` to enter the container before executing ROS 2 commands. | The `my_first_robot_package` directory and its associated files are successfully generated. |


## Analysis and Discussion (Section 1.11)

### 1. Build Time Comparison and Docker Caching
In the experiment, the first image build without cache took 111.5 seconds, while the second build with cache took only 1.8 seconds. This massive reduction occurs because Docker builds images in layers. During the first build, Docker executes every instruction in the Dockerfile and saves the result. On subsequent builds, if an instruction and its inputs haven't changed, Docker reuses the saved layer instead of re-executing the command. 

To optimize this caching mechanism, instructions in a Dockerfile should be ordered from least likely to change to most likely to change. For example, installing heavy dependencies (like `apt-get install python3-colcon-common`) should happen early in the file. Copying source code (`COPY src/ /ws/src/`) should happen at the very end. If source code is copied early, any minor code edit will invalidate the cache for all subsequent layers, forcing Docker to unnecessarily reinstall dependencies every time the code changes.

### 2. Trade-offs: Image Size vs. Tool Completeness
Choosing between `ros-base` and `ros-desktop` involves a strict trade-off between footprint and convenience. The `ros-base` image used in this module resulted in a final container size of 1.89 GB. It contains only the bare minimum libraries for communication and package building. This is highly efficient for deployment on robots with limited storage (like a Raspberry Pi on a TurtleBot3) and allows for faster pull/build times over the network.

Conversely, `ros-desktop` includes heavyweight GUI tools (like RViz and Rqt) and simulation environments (like Gazebo). While this provides a complete, out-of-the-box development environment, it inflates the image size significantly (often exceeding 3-4 GB). In this lab, we maintained a smaller image size by using `ros-base` and forwarding the X11 display server to the host when lightweight GUI tools were occasionally needed.

### 3. Network Mode, IPC Host, and Security Consequences
In the `compose.yaml` file, `network_mode: host` and `ipc: host` are used to bypass Docker's default network isolation. ROS 2 relies on the Data Distribution Service (DDS) protocol for node discovery, which operates using UDP multicast and shared memory. If the container uses default bridge networking, DDS discovery packets cannot easily reach nodes running on other machines (or other containers). By setting these to `host`, the container shares the host machine's IP address, network interfaces, and Inter-Process Communication (IPC) memory space, ensuring seamless ROS 2 communication.

The primary security consequence is the total loss of network isolation. A compromised container can listen to all network traffic on the host machine, access local host services, and manipulate shared memory segments. The container is no longer a securely isolated sandbox from a networking perspective.

### 4. Build Artifacts and `docker compose down -v`
During development, the `colcon build` command generates artifacts in the `build/`, `install/`, and `log/` directories, which are stored inside Docker named volumes (e.g., `ws_build` and `ws_install`). 

When the command `docker compose down -v` is executed, Docker stops the containers, removes them, and critically, deletes all associated named volumes. Consequently, all compiled binaries and setup scripts inside `build/` and `install/` are permanently destroyed. The source code (in `src/`) survives because it is stored in a bind mount linked to the host's hard drive, but the workspace must be completely rebuilt using `colcon build` the next time the container is brought up.


## Post-Lab Questions (Section 1.12)

**1. Perbedaan antara Docker image, container, dan layer:**
*   **Image:** A read-only template containing the OS, libraries, and source code needed to run an application. In this lab, `robotics-lab:dev` is an image.
*   **Container:** A running, isolated instance of an image. When we run `docker compose up`, we create a container named `robotics_dev` from our image.
*   **Layer:** An intermediate file system modification stacked to build an image. For example, the `RUN apt-get update...` command in our Dockerfile creates a distinct layer on top of the base Ubuntu layer.

**2. Mengapa kontainerisasi meningkatkan reproduktibilitas:**
Containerization packages the exact OS (Ubuntu 22.04) and dependencies (ROS 2 Humble) into a single environment. This guarantees that regardless of the host machine's OS (Windows WSL, macOS, or native Linux), all developers in a multi-computer project are running identically configured environments, eliminating "it works on my machine" errors.

**3. Risiko menggunakan tag image tanpa versi eksplisit (seperti `latest`):**
The `latest` tag is a moving target. Today it might point to ROS 2 Humble, but next year it might point to ROS 2 Jazzy. If the base image updates unexpectedly, it can break existing code due to deprecated libraries or structural changes, destroying the reproducibility of the environment.

**4. Kapan menggunakan bind mount vs. named volume:**
*   **Bind Mount (`src/`):** Use when you need direct, two-way file sharing between the host and container. This is ideal for source code (`src/`) so developers can use their host IDE (like VS Code) to write code, with changes instantly reflected inside the container.
*   **Named Volume (`install/`):** Use when data needs to persist but should remain isolated from the host OS. This is ideal for build artifacts (`install/`, `build/`) because compiling Linux binaries directly onto a Windows/Mac host drive can cause architecture conflicts and pollute the host workspace.

**5. Fungsi `source install/setup.bash` dan environment variable yang diubah:**
This command overlays the newly built workspace on top of the underlying ROS 2 installation. It modifies environment variables like `PATH` (to find executables), `PYTHONPATH` (to find Python modules), and `AMENT_PREFIX_PATH` (to locate ROS 2 packages), allowing the terminal to recognize and run the newly compiled `my_first_robot_package`.

**6. Analisis masalah: "Kode saya hilang setelah container direstart":**
The root cause is that the group did not properly mount a volume (neither bind mount nor named volume) to their workspace directory in their `compose.yaml`. Any code written directly to a container's default read-write layer is ephemeral and is permanently destroyed when the container is taken down and recreated.

**7. Mengapa menjalankan container sebagai root menyebabkan masalah kepemilikan file:**
If the container runs as `root`, any file it creates inside a bind mount (such as a new ROS 2 package or log file) will be assigned `root` ownership on the host machine's hard drive. The host's standard user will then be locked out from editing, moving, or deleting those files without using `sudo`. This was prevented in our lab by explicitly creating and logging in as the `dev` user.
