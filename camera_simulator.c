/*
 * camera_simulator.c
 * ------------------
 * Simulates a Camera sensor in the Multi-Sensor Fusion Pipeline.
 *
 * What this program does:
 *   1. Opens the named pipe (FIFO) "camera_fifo" for writing.
 *   2. Runs a scripted sequence of detections (deterministic test scenarios).
 *   3. Optionally switches to random simulation mode.
 *   4. Writes CameraDetection structs into the FIFO so the Fusion
 *      process can read them.
 *   5. Handles Ctrl+C (SIGINT) for clean shutdown.
 *
 * Compile:  gcc -Wall -Wextra -o camera_simulator camera_simulator.c
 * Run:      ./camera_simulator          (scripted mode — default)
 *           ./camera_simulator random   (random mode)
 *
 * The FIFO must already exist:
 *   mkfifo camera_fifo
 *
 * IPC Note:
 *   A FIFO open() for writing will BLOCK until the Fusion process
 *   opens the other end for reading.  Start fusion.c first.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#include "common.h"

/* =========================================================
 * Global state
 * ========================================================= */
static volatile int g_running = 1;
static int          g_fifo_fd = -1;

/* =========================================================
 * Signal handler
 * ========================================================= */
static void signal_handler(int sig)
{
    if (sig == SIGPIPE) {
        fprintf(stderr, "\n[Camera] Broken pipe — Fusion process may have closed.\n");
    } else {
        fprintf(stderr, "\n[Camera] Caught signal %d. Shutting down...\n", sig);
    }
    g_running = 0;
}

/* =========================================================
 * send_detection()
 * =========================================================
 * Stamps the detection, prints it, and writes it to the FIFO.
 */
static int send_detection(CameraDetection *det)
{
    /* Stamp with current time */
    camera_stamp(det);

    /* Print to terminal */
    printf("[Camera] Sending detection:\n");
    printf("         Object ID   : %u\n",    det->object_id);
    printf("         Object Type : %s\n",    OBJECT_TYPE_LABELS[det->object_type]);
    printf("         Angle       : %.1f deg\n", det->angle_deg);
    printf("         Size        : %s\n",    OBJECT_SIZE_LABELS[det->size]);
    printf("         Timestamp   : %ld.%03u\n\n",
           (long)det->timestamp, det->timestamp_ms);
    fflush(stdout);

    /* Write struct into FIFO */
    ssize_t written = write(g_fifo_fd, det, sizeof(CameraDetection));
    if (written != (ssize_t)sizeof(CameraDetection)) {
        if (errno == EPIPE) {
            fprintf(stderr, "[Camera] Broken pipe. Fusion process closed the FIFO.\n");
        } else {
            perror("[Camera] write() failed");
        }
        return -1;
    }
    return 0;
}

/* =========================================================
 * Scripted Scenario Sequence
 * =========================================================
 * These are designed to pair with the LiDAR scripted sequence.
 *
 * The camera sends at CAMERA_INTERVAL_MS = 600 ms.
 * The LiDAR sends at LIDAR_INTERVAL_MS   = 800 ms.
 *
 * Scenario coverage:
 *   Step 0 → Test 1: Object Person at 32° (matches LiDAR 30°)
 *   Step 1 → Test 3: Camera-only (Car at 110°, no LiDAR near 110°)
 *   Step 2 → Test 4: Different objects — Camera at 80°, LiDAR will be at 20°
 *   Step 3 → Test 1 repeat: Bicycle at 44° (matches LiDAR 45°)
 *   Step 4 → (no match for stale LiDAR detection at 10°)
 *             Camera sends Truck at 130° — different object entirely
 *   Step 5 → Matches LiDAR at -15° — Person at -13°
 *   Step 6 → Camera-only: Animal at 170°
 */
static const CameraDetection SCRIPTED_DETECTIONS[] = {
    /* Step 0 — Test 1: matches LiDAR Object 1 at 30° */
    { .object_id = 1, .object_type = OBJ_PERSON,  .angle_deg =  32.0f, .size = SIZE_MEDIUM },
    /* Step 1 — Test 3: Camera-only (no LiDAR near 110°) */
    { .object_id = 2, .object_type = OBJ_CAR,     .angle_deg = 110.0f, .size = SIZE_LARGE  },
    /* Step 2 — Test 4: Different objects — camera at 80° */
    { .object_id = 3, .object_type = OBJ_TRUCK,   .angle_deg =  80.0f, .size = SIZE_LARGE  },
    /* Step 3 — Test 1 repeat: matches LiDAR at 45° */
    { .object_id = 4, .object_type = OBJ_BICYCLE, .angle_deg =  44.0f, .size = SIZE_SMALL  },
    /* Step 4 — No match for stale LiDAR at 10° */
    { .object_id = 5, .object_type = OBJ_TRUCK,   .angle_deg = 130.0f, .size = SIZE_LARGE  },
    /* Step 5 — Matches LiDAR at -15° */
    { .object_id = 6, .object_type = OBJ_PERSON,  .angle_deg = -13.0f, .size = SIZE_MEDIUM },
    /* Step 6 — Camera-only: Animal at 170° */
    { .object_id = 7, .object_type = OBJ_ANIMAL,  .angle_deg = 170.0f, .size = SIZE_SMALL  },
};
#define SCRIPTED_COUNT  (int)(sizeof(SCRIPTED_DETECTIONS) / sizeof(SCRIPTED_DETECTIONS[0]))

/* =========================================================
 * Random simulation helpers
 * ========================================================= */
static float random_float(float min, float max)
{
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

static ObjectType random_type(void)
{
    return (ObjectType)((rand() % 5) + 1); /* OBJ_PERSON..OBJ_ANIMAL */
}

static ObjectSize random_size(void)
{
    return (ObjectSize)(rand() % 3);
}

/* =========================================================
 * main()
 * ========================================================= */
int main(int argc, char *argv[])
{
    int random_mode = 0;

    if (argc >= 2 && strcmp(argv[1], "random") == 0) {
        random_mode = 1;
        srand((unsigned int)time(NULL) + 42); /* Different seed from LiDAR */
        printf("[Camera] Starting in RANDOM simulation mode.\n\n");
    } else {
        printf("[Camera] Starting in SCRIPTED simulation mode.\n");
        printf("[Camera] (Run with argument 'random' for random mode)\n\n");
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, signal_handler);

    /* --------------------------------------------------
     * Open the FIFO for writing.
     * Blocks until Fusion process opens the read end.
     * -------------------------------------------------- */
    printf("[Camera] Opening FIFO '%s' for writing...\n", CAMERA_FIFO_PATH);
    printf("[Camera] (Waiting for Fusion process to open the other end)\n");
    fflush(stdout);

    g_fifo_fd = open(CAMERA_FIFO_PATH, O_WRONLY);
    if (g_fifo_fd < 0) {
        perror("[Camera] Failed to open FIFO");
        fprintf(stderr, "[Camera] Hint: Make sure you ran: mkfifo %s\n",
                CAMERA_FIFO_PATH);
        return EXIT_FAILURE;
    }
    printf("[Camera] FIFO opened. Starting to send detections...\n\n");

    /* --------------------------------------------------
     * Main send loop
     * -------------------------------------------------- */
    int script_index = 0;
    uint32_t next_id = 200;

    while (g_running) {
        CameraDetection det;
        memset(&det, 0, sizeof(det));

        if (random_mode) {
            det.object_id   = next_id++;
            det.object_type = random_type();
            det.angle_deg   = random_float(-90.0f, 90.0f);
            det.size        = random_size();
        } else {
            det = SCRIPTED_DETECTIONS[script_index % SCRIPTED_COUNT];
            script_index++;
        }

        if (send_detection(&det) < 0) {
            break;
        }

        struct timespec ts = {
    .tv_sec = CAMERA_INTERVAL_MS / 1000,
    .tv_nsec = (long)(CAMERA_INTERVAL_MS % 1000) * 1000000L
};
nanosleep(&ts, NULL);
    }

    if (g_fifo_fd >= 0) {
        close(g_fifo_fd);
    }
    printf("[Camera] Simulator stopped cleanly.\n");
    return EXIT_SUCCESS;
}
