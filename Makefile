# =============================================================
# Makefile — Multi-Sensor Fusion Pipeline
# =============================================================
#
# Usage:
#   make              — build all three programs
#   make fusion       — build only the fusion process
#   make lidar        — build only the LiDAR simulator
#   make camera       — build only the camera simulator
#   make fifos        — create the named pipes (FIFOs)
#   make clean        — remove binaries, FIFOs, and log files
#   make cleanlog     — remove only the log file
#
# Requires: GCC, Linux (Ubuntu 20.04+ recommended)
# =============================================================

CC      = gcc
CFLAGS  = -Wall -Wextra -std=c11 -pedantic -g
LDFLAGS = -lm

# Output binary names
TARGETS = fusion lidar_simulator camera_simulator

# Source files
FUSION_SRC  = fusion.c
LIDAR_SRC   = lidar_simulator.c
CAMERA_SRC  = camera_simulator.c

# Common header (if it changes, recompile everything)
COMMON_HDR  = common.h

# FIFO paths (must match LIDAR_FIFO_PATH / CAMERA_FIFO_PATH in common.h)
LIDAR_FIFO  = lidar_fifo
CAMERA_FIFO = camera_fifo

# Log file
LOG_FILE    = logs/fusion.log

# =============================================================
# Default target — build all programs
# =============================================================
.PHONY: all
all: $(TARGETS)
	@echo ""
	@echo "Build complete. Binaries: fusion  lidar_simulator  camera_simulator"
	@echo ""
	@echo "Quick-start:"
	@echo "  1. make fifos                    (create named pipes)"
	@echo "  2. ./fusion                      (Terminal 1)"
	@echo "  3. ./lidar_simulator             (Terminal 2)"
	@echo "  4. ./camera_simulator            (Terminal 3)"
	@echo ""

# =============================================================
# Individual targets
# =============================================================
fusion: $(FUSION_SRC) $(COMMON_HDR)
	$(CC) $(CFLAGS) -o $@ $(FUSION_SRC) $(LDFLAGS)
	@echo "[Makefile] Built: fusion"

lidar_simulator: $(LIDAR_SRC) $(COMMON_HDR)
	$(CC) $(CFLAGS) -o $@ $(LIDAR_SRC) $(LDFLAGS)
	@echo "[Makefile] Built: lidar_simulator"

camera_simulator: $(CAMERA_SRC) $(COMMON_HDR)
	$(CC) $(CFLAGS) -o $@ $(CAMERA_SRC) $(LDFLAGS)
	@echo "[Makefile] Built: camera_simulator"

# Convenience aliases
.PHONY: lidar camera
lidar:  lidar_simulator
camera: camera_simulator

# =============================================================
# Create FIFOs
# =============================================================
.PHONY: fifos
fifos:
	@echo "[Makefile] Creating named pipes (FIFOs)..."
	@if [ ! -p $(LIDAR_FIFO) ]; then \
		mkfifo $(LIDAR_FIFO) && echo "[Makefile] Created: $(LIDAR_FIFO)"; \
	else \
		echo "[Makefile] Already exists: $(LIDAR_FIFO)"; \
	fi
	@if [ ! -p $(CAMERA_FIFO) ]; then \
		mkfifo $(CAMERA_FIFO) && echo "[Makefile] Created: $(CAMERA_FIFO)"; \
	else \
		echo "[Makefile] Already exists: $(CAMERA_FIFO)"; \
	fi
	@echo "[Makefile] FIFOs ready."

# =============================================================
# Clean
# =============================================================
.PHONY: clean
clean:
	@echo "[Makefile] Removing binaries..."
	rm -f $(TARGETS)
	@echo "[Makefile] Removing FIFOs..."
	rm -f $(LIDAR_FIFO) $(CAMERA_FIFO)
	@echo "[Makefile] Removing log files..."
	rm -f $(LOG_FILE)
	@echo "[Makefile] Clean complete."

# Remove only the log file (keep binaries and FIFOs)
.PHONY: cleanlog
cleanlog:
	rm -f $(LOG_FILE)
	@echo "[Makefile] Log file removed."
