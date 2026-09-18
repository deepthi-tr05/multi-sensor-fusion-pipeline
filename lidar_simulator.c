/*
 * lidar_simulator.c
 * -----------------
 * Simulates a LiDAR sensor in the Multi-Sensor Fusion Pipeline.
 *
 * What this program does:
 *   1. Opens the named pipe (FIFO) "lidar_fifo" for writing.
 *   2. Runs a scripted sequence of detections (deterministic test scenarios).
 *   3. Optionally switches to random simulation mode.
 *   4. Writes LidarDetection structs into the FIFO so the Fusion
 *      process can read them.
 *   5. Handles Ctrl+C (SIGINT) for clean shutdown.
 *
 * Compile:  gcc -Wall -Wextra -o lidar_simulator lidar_simulator.c
 * Run:      ./lidar_simulator          (scripted mode — default)
 *           ./lidar_simulator random   (random mode)
 *
 * The FIFO must already exist:
 *   mkfifo lidar_fifo
 *
 * IPC Note:
 *   A FIFO open() for writing will BLOCK until the other end
 *   (fusion process) opens it for reading. Start fusion.c first.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>       /* write(), close(), usleep()  */
#include <fcntl.h>        /* open(), O_WRONLY            */
#include <signal.h>       /* signal(), SIGINT, SIGPIPE   */
#include <errno.h>
#include <time.h>

#include "common.h"

/* =========================================================
 * Global state
 * ========================================================= */
static volatile int g_running = 1;   /* Set to 0 by signal handler */
static int          g_fifo_fd = -1;  /* FIFO file descriptor        */

/* =========================================================
 * Signal handler
 * =========================================================
 * Handles Ctrl+C (SIGINT) and broken-pipe (SIGPIPE).
 * Sets g_running = 0 so the main loop exits cleanly.
 */
static void signal_handler(int sig)
{
    if (sig == SIGPIPE) {
        fprintf(stderr, "\n[LiDAR] Broken pipe — Fusion process may have closed.\n");
    } else {
        fprintf(stderr, "\n[LiDAR] Caught signal %d. Shutting down...\n", sig);
    }
    g_running = 0;
}

/* =========================================================
 * send_detection()
 * =========================================================
 * Stamps the detection with the current time, prints it to
 * the terminal, and writes the binary struct into the FIFO.
 *
 * Returns 0 on success, -1 on write failure.
 */
static int send_detection(LidarDetection *det)
{
    /* Stamp with current time */
    lidar_stamp(det);

    /* Print to terminal so the user can see what was sent */
    printf("[LiDAR] Sending detection:\n");
    printf("        Object ID : %u\n",      det->object_id);
    printf("        Distance  : %.1f m\n",  det->distance_m);
    printf("        Angle     : %.1f deg\n", det->angle_deg);
    printf("        Timestamp : %ld.%03u\n\n",
           (long)det->timestamp, det->timestamp_ms);
    fflush(stdout);

    /* Write the struct as binary data into the FIFO */
    ssize_t written = write(g_fifo_fd, det, sizeof(LidarDetection));
    if (written != (ssize_t)sizeof(LidarDetection)) {
        if (errno == EPIPE) {
            fprintf(stderr, "[LiDAR] Broken pipe. Fusion process closed the FIFO.\n");
        } else {
            perror("[LiDAR] write() failed");
        }
        return -1;
    }
    return 0;
}

/* =========================================================
 * Scripted Scenario Sequence
 * =========================================================
 * These detections map directly to the test scenarios in
 * test_scenarios.txt. Running the project in default mode
 * will always produce the same output, making it easy to
 * demonstrate during a viva.
 *
 * Scenario coverage:
 *   Step 0  → Test 1: Matching detection (angle 30°, matches camera 32°)
 *   Step 1  → Test 2: LiDAR-only (angle 60°, camera will not match)
 *   Step 2  → Test 4: Different objects (LiDAR 20°, camera will be at 80°)
 *   Step 3  → Test 1 again (second matching pair)
 *   Step 4  → Test 5: Stale data (we force an old timestamp after sending)
 *             The simulator sends a valid detection; to demonstrate the
 *             stale-discard path the fusion process uses STALE_THRESHOLD_MS.
 *             A normal detection at step 4 with no follow-up camera
 *             message will age out naturally.
 *   Steps 5+→ Test 1 repeat to keep the demo running
 */
static const LidarDetection SCRIPTED_DETECTIONS[] = {
    /* Step 0 — Test 1: matches camera Object 1 at ~32° */
    { .object_id = 1, .distance_m = 10.2f, .angle_deg = 30.0f },
    /* Step 1 — Test 2: LiDAR-only (camera will send nothing near 60°) */
    { .object_id = 2, .distance_m = 15.0f, .angle_deg = 60.0f },
    /* Step 2 — Test 4: Different objects — LiDAR at 20°, camera at 80° */
    { .object_id = 3, .distance_m =  8.5f, .angle_deg = 20.0f },
    /* Step 3 — Test 1 repeat: another close match */
    { .object_id = 4, .distance_m = 12.0f, .angle_deg = 45.0f },
    /* Step 4 — Will become stale (camera won't match; ages past threshold) */
    { .object_id = 5, .distance_m = 25.0f, .angle_deg = 10.0f },
    /* Step 5 — Continue normal matching */
    { .object_id = 6, .distance_m =  5.0f, .angle_deg = -15.0f },
    /* Step 6 — Normal */
    { .object_id = 7, .distance_m = 18.0f, .angle_deg =  90.0f },
};
#define SCRIPTED_COUNT  (int)(sizeof(SCRIPTED_DETECTIONS) / sizeof(SCRIPTED_DETECTIONS[0]))

/* =========================================================
 * Random simulation helpers
 * ========================================================= */
static float random_float(float min, float max)
{
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

/* =========================================================
 * main()
 * ========================================================= */
int main(int argc, char *argv[])
{
    int random_mode = 0;

    /* Parse optional "random" argument */
    if (argc >= 2 && strcmp(argv[1], "random") == 0) {
        random_mode = 1;
        srand((unsigned int)time(NULL));
        printf("[LiDAR] Starting in RANDOM simulation mode.\n\n");
    } else {
        printf("[LiDAR] Starting in SCRIPTED simulation mode.\n");
        printf("[LiDAR] (Run with argument 'random' for random mode)\n\n");
    }

    /* Register signal handlers */
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, signal_handler);

    /* --------------------------------------------------
     * Open the FIFO for writing.
     *
     * IMPORTANT: open() on a write-only FIFO BLOCKS until
     * the reader (fusion process) opens the other end.
     * This is normal FIFO behaviour — start fusion.c first.
     * -------------------------------------------------- */
    printf("[LiDAR] Opening FIFO '%s' for writing...\n", LIDAR_FIFO_PATH);
    printf("[LiDAR] (Waiting for Fusion process to open the other end)\n");
    fflush(stdout);

    g_fifo_fd = open(LIDAR_FIFO_PATH, O_WRONLY);
    if (g_fifo_fd < 0) {
        perror("[LiDAR] Failed to open FIFO");
        fprintf(stderr, "[LiDAR] Hint: Make sure you ran: mkfifo %s\n",
                LIDAR_FIFO_PATH);
        return EXIT_FAILURE;
    }
    printf("[LiDAR] FIFO opened. Starting to send detections...\n\n");

    /* --------------------------------------------------
     * Main send loop
     * -------------------------------------------------- */
    int script_index = 0;
    uint32_t next_id = 100; /* ID counter for random mode */

    while (g_running) {
        LidarDetection det;
        memset(&det, 0, sizeof(det));

        if (random_mode) {
            /* Random detection */
            det.object_id  = next_id++;
            det.distance_m = random_float(2.0f, 50.0f);
            det.angle_deg  = random_float(-90.0f, 90.0f);
        } else {
            /* Scripted detection — cycle through the sequence */
            det = SCRIPTED_DETECTIONS[script_index % SCRIPTED_COUNT];
            script_index++;
        }

        /* Send — exit loop if write fails (fusion closed the pipe) */
        if (send_detection(&det) < 0) {
            break;
        }

        /* Sleep for LIDAR_INTERVAL_MS milliseconds */
        struct timespec ts = {
    .tv_sec = LIDAR_INTERVAL_MS / 1000,
    .tv_nsec = (long)(LIDAR_INTERVAL_MS % 1000) * 1000000L
};
nanosleep(&ts, NULL);
    }

    /* --------------------------------------------------
     * Clean up
     * -------------------------------------------------- */
    if (g_fifo_fd >= 0) {
        close(g_fifo_fd);
    }
    printf("[LiDAR] Simulator stopped cleanly.\n");
    return EXIT_SUCCESS;
}
