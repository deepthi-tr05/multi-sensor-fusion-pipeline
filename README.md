# Multi-Sensor Fusion Pipeline Using IPC

A Linux C programming project that simulates an automotive multi-sensor
fusion pipeline using Named Pipes (FIFOs) for Inter-Process Communication (IPC).

---

## 1. Project Objective

Modern autonomous vehicles use multiple sensors to detect and understand
their surroundings. No single sensor is perfect:

- **LiDAR** measures distance accurately but cannot identify what it sees.
- **Cameras** can classify objects (Person, Car, Truck) but struggle with
  depth estimation in poor lighting.

By **fusing** data from both sensors, we get a more reliable, richer
picture of the environment — combining the distance accuracy of LiDAR
with the classification capability of Camera.

This project simulates that pipeline in software using three independent
Linux processes communicating through Named Pipes.

---

## 2. Real-Life Automotive Example

Imagine a car driving at night:

- The **LiDAR** detects something at 10.2 m, 30 degrees ahead.
- The **Camera** sees a Person at roughly 32 degrees.
- The **Fusion Process** compares them: only 2 degrees apart — same object!
- Result: *Person detected at 10.2 m, 31 degrees, Confidence 95%.*

The driver-assistance system can now brake confidently because two
independent sensors agree on the same obstacle.

---

## 3. System Architecture

```
  ┌─────────────────────┐        lidar_fifo (FIFO)       ┌───────────────────┐
  │   lidar_simulator   │ ──────────────────────────────► │                   │
  │   (Process 1)       │                                 │   fusion          │
  └─────────────────────┘                                 │   (Process 3)     │
                                                          │                   │
  ┌─────────────────────┐       camera_fifo (FIFO)        │  • select()       │
  │  camera_simulator   │ ──────────────────────────────► │  • association    │
  │   (Process 2)       │                                 │  • stale check    │
  └─────────────────────┘                                 │  • logging        │
                                                          └───────────────────┘
```

Three independent programs, three independent processes. Communication is
only through the two named pipes — no shared memory, no sockets, no threads.

---

## 4. What the LiDAR Simulator Does

**File:** `lidar_simulator.c`

- Sends a `LidarDetection` struct into `lidar_fifo` every 800 ms.
- Each detection contains: Object ID, Distance (metres), Angle (degrees),
  Timestamp.
- **Scripted mode** (default): plays a fixed 7-step sequence that covers
  all test scenarios — same output every run, ideal for demonstrations.
- **Random mode**: generates random distances and angles.

```
./lidar_simulator           # scripted
./lidar_simulator random    # random
```

---

## 5. What the Camera Simulator Does

**File:** `camera_simulator.c`

- Sends a `CameraDetection` struct into `camera_fifo` every 600 ms.
- Each detection contains: Object ID, Object Type (Person/Car/Truck/…),
  Angle (degrees), Size (Small/Medium/Large), Timestamp.
- Same scripted/random modes as the LiDAR simulator.
- The scripted sequence is designed to pair with the LiDAR sequence so
  that all five test scenarios are observable.

```
./camera_simulator          # scripted
./camera_simulator random   # random
```

---

## 6. What the Fusion Process Does

**File:** `fusion.c`

1. Creates `lidar_fifo` and `camera_fifo` if they do not exist.
2. Opens both FIFOs with `O_RDONLY | O_NONBLOCK`.
3. Uses **`select()`** with a 500 ms timeout to monitor both FIFOs
   simultaneously — neither sensor can starve the other.
4. When a detection arrives, it searches the opposite sensor's pending
   buffer for a matching detection using the angle-tolerance rule.
5. If a match is found → prints a **Fused Object**.
6. If no match → stores the detection in a pending buffer.
7. Every 500 ms → scans pending buffers and discards anything older than
   `STALE_THRESHOLD_MS` (2000 ms).
8. Logs all events to `logs/fusion.log`.
9. On Ctrl+C → reports any remaining unmatched detections and exits cleanly.

---

## 7. What is IPC?

**IPC = Inter-Process Communication.**

In Linux, processes are isolated — they cannot directly read each other's
memory. IPC is the collection of mechanisms the OS provides for processes
to exchange data. Examples include:

| Mechanism      | This project uses? |
|----------------|--------------------|
| Named Pipes (FIFOs) | ✅ Yes        |
| Shared Memory  | ❌ No              |
| Sockets        | ❌ No              |
| Message Queues | ❌ No              |
| Signals        | ✅ Yes (Ctrl+C)    |

---

## 8. Why Named Pipes / FIFOs?

A **Named Pipe (FIFO)** is a special file in the filesystem that acts as
a one-way data channel between processes:

- Created with `mkfifo` or the `mkfifo()` system call.
- One process **writes** to it; another **reads** from it.
- Data flows in order (FIFO = First In, First Out).
- Simple to use — reads and writes use standard `read()`/`write()` calls.
- Visible in the filesystem (`ls -l` shows them with `p` type).
- Perfect for this project because the sensor simulators and fusion process
  are completely separate, independent programs.

**Why not shared memory or sockets?**
Named pipes are the simplest, most educational IPC mechanism for a
producer–consumer pipeline. They map naturally to the concept of a
sensor data stream.

---

## 9. How Sensor Association Works

The fusion process uses a **simple angle-tolerance rule**:

```
Two detections represent the same object if:
  |lidar.angle_deg - camera.angle_deg| <= ANGLE_TOLERANCE_DEG
```

`ANGLE_TOLERANCE_DEG` is defined in `common.h` (default: **5.0 degrees**).

### Example — Match:

```
LiDAR angle  = 30.0 deg
Camera angle = 32.0 deg
Difference   =  2.0 deg  <=  5.0  →  SAME OBJECT
```

### Example — No Match:

```
LiDAR angle  = 30.0 deg
Camera angle = 80.0 deg
Difference   = 50.0 deg  >>  5.0  →  DIFFERENT OBJECTS
```

### Confidence Score (rule-based):

| Condition                          | Points |
|------------------------------------|--------|
| Base score                         |  +70   |
| Angle difference < 1.0 deg         |  +15   |
| Angle difference < 3.0 deg         |  +10   |
| Angle difference < 5.0 deg         |   +5   |
| Camera type is not Unknown         |  +10   |
| Camera size is Medium or Large     |   +5   |
| **Maximum (capped)**               | **99** |

Only the best angle bracket applies (not cumulative).

---

## 10. How Stale Data is Handled

Every detection is timestamped when it is created (using `gettimeofday()`).

The fusion process runs a **stale check after every `select()` call**
(at least every 500 ms):

```
For each pending detection:
  age = current_time_ms - detection_timestamp_ms
  if age > STALE_THRESHOLD_MS (2000 ms):
      report as single-sensor detection
      discard from pending buffer
```

This prevents old, unmatched detections from hanging around indefinitely
and incorrectly matching future detections from a completely different object.

---

## 11. Project Files

```
multi_sensor_fusion/
├── common.h              Shared structs and constants
├── lidar_simulator.c     LiDAR sensor simulation
├── camera_simulator.c    Camera sensor simulation
├── fusion.c              Fusion process (select, match, log)
├── Makefile              Build rules
├── README.md             This file
├── test_scenarios.txt    Test scenario descriptions and expected output
└── logs/
    └── fusion.log        Runtime log (created when fusion runs)
```

---

## 12. Compilation

### Prerequisites

- GCC (any version supporting C11)
- Linux (Ubuntu 20.04+ recommended)
- Standard C library only — no external dependencies

### Build

```bash
# From inside the multi_sensor_fusion/ directory:
make
```

This compiles all three programs with `-Wall -Wextra -std=c11 -pedantic`.

To build individually:
```bash
make fusion
make lidar_simulator
make camera_simulator
```

---

## 13. Execution — Step by Step

### Step 1: Create the Named Pipes

```bash
make fifos
```

Or manually:
```bash
mkfifo lidar_fifo
mkfifo camera_fifo
```

Verify they were created:
```bash
ls -l lidar_fifo camera_fifo
# Should show: prw-rw-r-- (p = named pipe)
```

### Step 2: Start the Fusion Process (Terminal 1 — always first)

```bash
./fusion
```

The fusion process creates the FIFOs (if missing) and opens them
in non-blocking mode. It prints:

```
=====================================================
   Multi-Sensor Fusion Process
   Angle Tolerance : 5.0 degrees
   Stale Threshold : 2000 ms
=====================================================

[FUSION] Created FIFO: lidar_fifo
[FUSION] Created FIFO: camera_fifo
[FUSION] Opening 'lidar_fifo' (non-blocking)...
[FUSION] Opening 'camera_fifo' (non-blocking)...
[FUSION] Both FIFOs opened. Waiting for sensor data...
```

### Step 3: Start the LiDAR Simulator (Terminal 2)

```bash
./lidar_simulator
```

Output:
```
[LiDAR] Starting in SCRIPTED simulation mode.
[LiDAR] Opening FIFO 'lidar_fifo' for writing...
[LiDAR] FIFO opened. Starting to send detections...

[LiDAR] Sending detection:
        Object ID : 1
        Distance  : 10.2 m
        Angle     : 30.0 deg
        Timestamp : 1700000000.123
```

### Step 4: Start the Camera Simulator (Terminal 3)

```bash
./camera_simulator
```

Output:
```
[Camera] Starting in SCRIPTED simulation mode.
[Camera] Opening FIFO 'camera_fifo' for writing...
[Camera] FIFO opened. Starting to send detections...

[Camera] Sending detection:
         Object ID   : 1
         Object Type : Person
         Angle       : 32.0 deg
         Size        : Medium
         Timestamp   : 1700000000.456
```

### Step 5: Observe Fusion Output (Terminal 1)

Shortly after both sensors send their first detections, Terminal 1
shows:

```
[FUSION] Matching LiDAR and Camera detections...

========== FUSED OBJECT ==========
Object Type : Person
Distance    : 10.2 m
Angle       : 31.0 degrees
Size        : Medium
Confidence  : 95%
LiDAR ID    : 1
Camera ID   : 1
===================================
```

---

## 14. Test Scenarios

See `test_scenarios.txt` for the full description.

| # | Scenario            | Expected Result             |
|---|---------------------|-----------------------------|
| 1 | LiDAR 30° + Cam 32° | Fused object, 95% confidence |
| 2 | LiDAR 60°, no cam  | LiDAR-only detection         |
| 3 | Cam 110°, no LiDAR | Camera-only detection        |
| 4 | LiDAR 20° + Cam 80° | NOT fused (60° difference)  |
| 5 | Unmatched, old data | Stale discard message        |

---

## 15. Expected Output Samples

### Fused Object
```
[FUSION] Matching LiDAR and Camera detections...

========== FUSED OBJECT ==========
Object Type : Person
Distance    : 10.2 m
Angle       : 31.0 degrees
Size        : Medium
Confidence  : 95%
LiDAR ID    : 1
Camera ID   : 1
===================================
```

### LiDAR-Only
```
[FUSION] Single-sensor detection
  Source   : LiDAR
  Object ID: 2
  Distance : 15.0 m
  Angle    : 60.0 degrees
```

### Camera-Only
```
[FUSION] Single-sensor detection
  Source      : Camera
  Object ID   : 2
  Object Type : Car
  Angle       : 110.0 degrees
  Size        : Large
```

### Stale Discard
```
[FUSION] Discarding stale LiDAR detection: ID=5 age=2134 ms
[FUSION] Single-sensor detection
  Source   : LiDAR
  Object ID: 5
  Distance : 25.0 m
  Angle    : 10.0 degrees
```

---

## 16. Stopping the Programs

Press **Ctrl+C** in each terminal window.

Recommended order: stop the sensor simulators first, then the fusion
process. But any order is safe — the programs handle broken pipes and
EOF gracefully.

---

## 17. Cleaning Up

```bash
make clean
```

This removes:
- Compiled binaries (`fusion`, `lidar_simulator`, `camera_simulator`)
- Named pipe files (`lidar_fifo`, `camera_fifo`)
- Log file (`logs/fusion.log`)

To remove only the log:
```bash
make cleanlog
```

---

## 18. Viewing the Log File

```bash
# View the full log:
cat logs/fusion.log

# Watch it live while processes are running:
tail -f logs/fusion.log
```

Sample log content:
```
============================================================
  Fusion Process Started
============================================================
[2024-01-15 10:30:01.234] [FUSION] Process started. PID=12345
[2024-01-15 10:30:01.235] [FUSION] Listening on lidar_fifo and camera_fifo...
[2024-01-15 10:30:02.100] [FUSION] Received LiDAR detection: ID=1 Dist=10.2m Angle=30.0deg
[2024-01-15 10:30:02.350] [FUSION] Received Camera detection: ID=1 Type=Person Angle=32.0deg Size=Medium
[2024-01-15 10:30:02.351] [FUSION] Matched: Camera_ID=1 (32.0deg) <-> LiDAR_ID=1 (30.0deg) diff=2.0deg
[2024-01-15 10:30:02.351] [FUSION] FUSED OBJECT
  Type=Person Distance=10.2m Angle=31.0deg Size=Medium Confidence=95% LiDAR_ID=1 Camera_ID=1
[2024-01-15 10:30:02.900] [FUSION] Received LiDAR detection: ID=2 Dist=15.0m Angle=60.0deg
[2024-01-15 10:30:03.500] [FUSION] Received Camera detection: ID=2 Type=Car Angle=110.0deg Size=Large
[2024-01-15 10:30:04.900] [FUSION] Discarding stale LiDAR detection: ID=2 age=2001 ms
[2024-01-15 10:30:04.901] [FUSION] SINGLE-SENSOR LiDAR: ID=2 Dist=15.0m Angle=60.0deg
```

---

## 19. Troubleshooting

### "Failed to open FIFO" / "No such file or directory"
```bash
make fifos        # or: mkfifo lidar_fifo camera_fifo
```

### LiDAR/Camera simulator hangs on startup (no output after "Waiting...")
- The simulator's `open()` is waiting for the Fusion process to open the
  other end of the FIFO.
- **Fix:** Make sure `./fusion` is running in Terminal 1 first.

### "Broken pipe" error
- The Fusion process closed before the simulator finished.
- Restart the Fusion process first, then restart the simulators.

### Fusion sees no data after simulators start
- Check that `lidar_fifo` and `camera_fifo` exist in the same directory.
- Check that all three programs are run from the same directory.

### "Permission denied" on FIFO
```bash
chmod 666 lidar_fifo camera_fifo
```

### Compilation errors
```bash
gcc --version       # Must be GCC 5 or later for C11 support
make clean && make  # Clean rebuild
```

### Log file not created
- Make sure the `logs/` directory exists (created by `make` target or
  by the fusion process automatically).
```bash
mkdir -p logs
```

---

## 20. Project Limitations

1. **Angle-only association** — the matching rule uses only angle.
   A real system would use 2D/3D position, velocity, and object size.

2. **No coordinate transformation** — LiDAR and Camera may have different
   mounting positions and fields of view on a real vehicle. This
   simulation assumes they share the same reference frame.

3. **Binary struct IPC** — the FIFO carries raw C structs. This works
   when all programs are compiled together on the same machine but is
   not portable across different architectures (endianness, alignment).

4. **Single-direction FIFOs** — the sensor simulators cannot receive
   feedback from the fusion process (e.g., no acknowledgement).

5. **No track management** — in a real fusion system, matched objects
   are tracked over time (Kalman filter, track ID persistence). This
   project treats each detection as independent.

6. **Scripted simulation only** — the simulated sensor data does not
   come from real hardware.

---

## 21. Possible Future Improvements

| Improvement              | Description |
|--------------------------|-------------|
| Kalman Filter tracking   | Smooth position/velocity estimates over time per track |
| 2D position fusion       | Use (x, y) coordinates instead of angle only |
| Track ID persistence     | Assign stable IDs to objects across multiple frames |
| JSON/text protocol       | Replace binary structs with a text format for portability |
| Bidirectional FIFOs      | Allow fusion to send commands back to simulators |
| Multiple sensor types    | Add radar, ultrasonic, GPS |
| Ncurses visualisation    | Draw a 2D overhead map of detected objects in the terminal |
| Unit tests               | Add a test harness using `assert()` or a lightweight C test framework |
| CMake build system       | Replace Makefile with CMake for cross-platform builds |
| ROS integration          | Publish fused objects as ROS topics for use with Gazebo simulation |

---

## 22. Key Concepts Summary (for Viva)

| Question | Answer |
|----------|--------|
| What is a FIFO? | A special file used as a one-directional data channel between processes |
| Why `O_NONBLOCK`? | Prevents `open()` from blocking; lets fusion open both FIFOs without waiting for writers |
| Why `select()`? | Monitors multiple file descriptors simultaneously; avoids starving either sensor |
| What is the association rule? | Match if angle difference ≤ ANGLE_TOLERANCE_DEG (5°) |
| How is confidence calculated? | Rule-based: base 70 + angle bonus + type bonus + size bonus, capped at 99 |
| How are stale detections handled? | Periodic age check; discard and report as single-sensor if older than 2000 ms |
| What happens when a sensor stops? | EOF on FIFO is detected; FIFO is reopened; stale timer cleans up pending data |
| How is Ctrl+C handled? | `SIGINT` sets a flag; main loop exits; remaining pending detections are reported; FDs closed |

---

*This project was developed as an academic Linux IPC programming exercise.*
*It demonstrates C system programming, process communication, and basic sensor fusion concepts.*
