/*
 * fusion.c
 * --------
 * The Fusion Process for the Multi-Sensor Fusion Pipeline.
 *
 * What this program does:
 *   1. Creates the two named pipes (FIFOs) if they do not exist.
 *   2. Opens both FIFOs for reading using O_RDONLY | O_NONBLOCK so
 *      that neither open() call blocks waiting for a writer.
 *   3. Uses Linux select() to monitor both FIFOs simultaneously —
 *      this prevents either sensor from starving the other.
 *   4. Receives LidarDetection and CameraDetection structs.
 *   5. Stores unmatched detections in pending arrays.
 *   6. Attempts to associate (match) new detections against pending
 *      detections from the other sensor using an angle-tolerance rule.
 *   7. When a match is found: produces a FusedObject and prints it.
 *   8. Periodically scans pending arrays and discards stale entries.
 *   9. When no match is found after STALE_THRESHOLD_MS: single-sensor
 *      detection is reported.
 *  10. Logs all significant events to logs/fusion.log.
 *  11. Handles Ctrl+C (SIGINT) for clean shutdown.
 *
 * Compile:  gcc -Wall -Wextra -o fusion fusion.c
 * Run:      ./fusion
 *
 * Start this process BEFORE starting the sensor simulators.
 *
 * ---------------------------------------------------------------
 * Sensor Association Rule (simple angle-tolerance method):
 *
 *   Two detections are the "same object" if:
 *       |lidar.angle_deg - camera.angle_deg| <= ANGLE_TOLERANCE_DEG
 *
 *   ANGLE_TOLERANCE_DEG is defined in common.h (default: 5.0 degrees).
 *
 * ---------------------------------------------------------------
 * Confidence Score Calculation (rule-based, no ML):
 *
 *   Base score = 70
 *   + 15  if angle difference < 1.0 deg  (very tight match)
 *   + 10  if angle difference < 3.0 deg  (good match)
 *   +  5  if angle difference < 5.0 deg  (acceptable match)
 *   + 10  if camera object_type != OBJ_UNKNOWN
 *   +  5  if camera size != SIZE_SMALL    (larger = easier to detect)
 *   Capped at 99.
 *
 * ---------------------------------------------------------------
 * Stale Data Handling:
 *
 *   Every time select() returns (or times out), the fusion process
 *   checks each pending LiDAR and Camera detection.
 *   If a detection's age > STALE_THRESHOLD_MS it is discarded and
 *   a "[FUSION] Discarding stale ..." message is printed and logged.
 * ---------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>   /* va_list, va_start, va_end, vprintf, vfprintf */
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>

#include "common.h"

/* =========================================================
 * Internal state
 * ========================================================= */

/* Pending (unmatched) detection buffers */
static LidarDetection  pending_lidar[MAX_PENDING];
static CameraDetection pending_camera[MAX_PENDING];
static int             pending_lidar_count  = 0;
static int             pending_camera_count = 0;

/* File descriptor for the log file */
static FILE *g_log_fp = NULL;

/* Signal flag */
static volatile int g_running = 1;

/* FIFO file descriptors */
static int g_lidar_fd  = -1;
static int g_camera_fd = -1;

/* =========================================================
 * Signal handler
 * ========================================================= */
static void signal_handler(int sig)
{
    /* Write signal number to stderr using async-signal-safe write().
     * fprintf is not async-signal-safe, but this is acceptable in a
     * student/demo context where the signal is always Ctrl+C / SIGTERM. */
    (void)sig;  /* suppress unused-parameter warning */
    g_running = 0;
}

/* =========================================================
 * Logging helpers
 * =========================================================
 * log_event() writes to both stdout and the log file.
 * All log lines are prefixed with a human-readable timestamp.
 */
static void log_event(const char *fmt, ...)
{
    /* Build timestamp string */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_info = localtime(&tv.tv_sec);
    char time_buf[32];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_info);

    va_list args;

    /* Print to stdout */
    printf("[%s.%03d] ", time_buf, (int)(tv.tv_usec / 1000));
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);

    /* Mirror to log file */
    if (g_log_fp) {
        fprintf(g_log_fp, "[%s.%03d] ", time_buf, (int)(tv.tv_usec / 1000));
        va_start(args, fmt);
        vfprintf(g_log_fp, fmt, args);
        va_end(args);
        fflush(g_log_fp);
    }
}

/* Simple log-only write (no stdout echo) */
static void log_only(const char *fmt, ...)
{
    if (!g_log_fp) return;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_info = localtime(&tv.tv_sec);
    char time_buf[32];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_info);

    fprintf(g_log_fp, "[%s.%03d] ", time_buf, (int)(tv.tv_usec / 1000));
    va_list args;
    va_start(args, fmt);
    vfprintf(g_log_fp, fmt, args);
    va_end(args);
    fflush(g_log_fp);
}

/* =========================================================
 * open_log()
 * =========================================================
 * Opens the log file for appending. Creates the logs/
 * directory if it does not exist.
 */
static void open_log(void)
{
    /* Ensure the logs directory exists */
    struct stat st;
    if (stat("logs", &st) != 0) {
        if (mkdir("logs", 0755) != 0 && errno != EEXIST) {
            perror("[FUSION] Warning: could not create logs/ directory");
            return;
        }
    }

    g_log_fp = fopen(LOG_FILE_PATH, "a");
    if (!g_log_fp) {
        perror("[FUSION] Warning: could not open log file");
    } else {
        fprintf(g_log_fp,
                "\n============================================================\n"
                "  Fusion Process Started\n"
                "============================================================\n");
        fflush(g_log_fp);
    }
}

/* =========================================================
 * create_fifos()
 * =========================================================
 * Creates the named pipes if they do not already exist.
 * mkfifo() returns EEXIST when the FIFO is already there —
 * that is not an error.
 */
static int create_fifos(void)
{
    const char *paths[] = { LIDAR_FIFO_PATH, CAMERA_FIFO_PATH };
    for (int i = 0; i < 2; i++) {
        if (mkfifo(paths[i], 0666) < 0) {
            if (errno == EEXIST) {
                printf("[FUSION] FIFO '%s' already exists — reusing.\n", paths[i]);
            } else {
                perror("[FUSION] mkfifo failed");
                fprintf(stderr, "[FUSION] Could not create FIFO: %s\n", paths[i]);
                return -1;
            }
        } else {
            printf("[FUSION] Created FIFO: %s\n", paths[i]);
        }
    }
    return 0;
}

/* =========================================================
 * open_fifos_nonblocking()
 * =========================================================
 * Opens both FIFOs with O_RDONLY | O_NONBLOCK.
 *
 * Why O_NONBLOCK?
 *   Normally, open() on a read-only FIFO blocks until a writer
 *   opens the other end. By using O_NONBLOCK we open immediately
 *   and let select() tell us when data is available. This lets
 *   the fusion process start and wait for BOTH sensors without
 *   being stuck on one FIFO open call.
 *
 * Returns 0 on success, -1 on failure.
 */
static int open_fifos_nonblocking(void)
{
    printf("[FUSION] Opening '%s' (non-blocking)...\n", LIDAR_FIFO_PATH);
    g_lidar_fd = open(LIDAR_FIFO_PATH, O_RDONLY | O_NONBLOCK);
    if (g_lidar_fd < 0) {
        perror("[FUSION] Failed to open lidar_fifo");
        return -1;
    }

    printf("[FUSION] Opening '%s' (non-blocking)...\n", CAMERA_FIFO_PATH);
    g_camera_fd = open(CAMERA_FIFO_PATH, O_RDONLY | O_NONBLOCK);
    if (g_camera_fd < 0) {
        perror("[FUSION] Failed to open camera_fifo");
        close(g_lidar_fd);
        g_lidar_fd = -1;
        return -1;
    }

    printf("[FUSION] Both FIFOs opened. Waiting for sensor data...\n\n");
    return 0;
}

/* =========================================================
 * compute_confidence()
 * =========================================================
 * Rule-based confidence score (0–99).
 * See header comment for full breakdown.
 */
static int compute_confidence(const LidarDetection  *ld,
                               const CameraDetection *cd)
{
    float diff = fabsf(ld->angle_deg - cd->angle_deg);
    int score = 70;

    /* Angle closeness bonus — only the best bracket applies */
    if (diff < 1.0f) {
        score += 15;
    } else if (diff < 3.0f) {
        score += 10;
    } else if (diff < 5.0f) {
        score += 5;
    }

    /* Camera classification quality */
    if (cd->object_type != OBJ_UNKNOWN) {
        score += 10;
    }

    /* Object size bonus — larger objects are more reliably detected */
    if (cd->size != SIZE_SMALL) {
        score += 5;
    }

    /* Cap at 99 — 100% confidence is never claimed */
    if (score > 99) score = 99;
    return score;
}

/* =========================================================
 * print_fused_object()
 * =========================================================
 * Prints the formatted fused result to stdout and log file.
 */
static void print_fused_object(const FusedObject *fused)
{
    const char *sep = "===================================";

    printf("\n[FUSION] Matching LiDAR and Camera detections...\n");
    printf("\n========== FUSED OBJECT ==========\n");
    printf("Object Type : %s\n",        OBJECT_TYPE_LABELS[fused->object_type]);
    printf("Distance    : %.1f m\n",    fused->distance_m);
    printf("Angle       : %.1f degrees\n", fused->angle_deg);
    printf("Size        : %s\n",        OBJECT_SIZE_LABELS[fused->size]);
    printf("Confidence  : %d%%\n",      fused->confidence_pct);
    printf("LiDAR ID    : %u\n",        fused->lidar_id);
    printf("Camera ID   : %u\n",        fused->camera_id);
    printf("%s\n\n", sep);
    fflush(stdout);

    if (g_log_fp) {
        fprintf(g_log_fp,
                "\n[FUSION] FUSED OBJECT\n"
                "  Type=%s Distance=%.1fm Angle=%.1fdeg "
                "Size=%s Confidence=%d%% LiDAR_ID=%u Camera_ID=%u\n",
                OBJECT_TYPE_LABELS[fused->object_type],
                fused->distance_m,
                fused->angle_deg,
                OBJECT_SIZE_LABELS[fused->size],
                fused->confidence_pct,
                fused->lidar_id,
                fused->camera_id);
        fflush(g_log_fp);
    }
}

/* =========================================================
 * print_lidar_only()
 * =========================================================
 * Called when a LiDAR detection has no camera match.
 */
static void print_lidar_only(const LidarDetection *ld)
{
    printf("\n[FUSION] Single-sensor detection\n");
    printf("  Source   : LiDAR\n");
    printf("  Object ID: %u\n",      ld->object_id);
    printf("  Distance : %.1f m\n", ld->distance_m);
    printf("  Angle    : %.1f degrees\n\n", ld->angle_deg);
    fflush(stdout);

    log_only("[FUSION] SINGLE-SENSOR LiDAR: ID=%u Dist=%.1fm Angle=%.1fdeg\n",
             ld->object_id, ld->distance_m, ld->angle_deg);
}

/* =========================================================
 * print_camera_only()
 * =========================================================
 * Called when a camera detection has no LiDAR match.
 */
static void print_camera_only(const CameraDetection *cd)
{
    printf("\n[FUSION] Single-sensor detection\n");
    printf("  Source      : Camera\n");
    printf("  Object ID   : %u\n",   cd->object_id);
    printf("  Object Type : %s\n",   OBJECT_TYPE_LABELS[cd->object_type]);
    printf("  Angle       : %.1f degrees\n", cd->angle_deg);
    printf("  Size        : %s\n\n", OBJECT_SIZE_LABELS[cd->size]);
    fflush(stdout);

    log_only("[FUSION] SINGLE-SENSOR Camera: ID=%u Type=%s Angle=%.1fdeg Size=%s\n",
             cd->object_id,
             OBJECT_TYPE_LABELS[cd->object_type],
             cd->angle_deg,
             OBJECT_SIZE_LABELS[cd->size]);
}

/* =========================================================
 * remove_lidar_pending()
 * =========================================================
 * Removes entry at index i from the pending LiDAR array by
 * shifting subsequent entries left (compact array).
 */
static void remove_lidar_pending(int i)
{
    for (int j = i; j < pending_lidar_count - 1; j++) {
        pending_lidar[j] = pending_lidar[j + 1];
    }
    pending_lidar_count--;
}

/* =========================================================
 * remove_camera_pending()
 * =========================================================
 * Same for camera pending array.
 */
static void remove_camera_pending(int i)
{
    for (int j = i; j < pending_camera_count - 1; j++) {
        pending_camera[j] = pending_camera[j + 1];
    }
    pending_camera_count--;
}

/* =========================================================
 * try_match_lidar()
 * =========================================================
 * A new LiDAR detection has arrived. Scan the pending camera
 * array for a match (within ANGLE_TOLERANCE_DEG).
 *
 * If a match is found:
 *   - Build a FusedObject, print and log it.
 *   - Remove the matched camera entry from pending.
 *   - Return 1 (matched).
 *
 * If no match:
 *   - Add the LiDAR detection to the pending LiDAR array.
 *   - Return 0 (unmatched — will be checked again on future reads).
 */
static int try_match_lidar(const LidarDetection *ld)
{
    for (int i = 0; i < pending_camera_count; i++) {
        float diff = fabsf(ld->angle_deg - pending_camera[i].angle_deg);
        if (diff <= ANGLE_TOLERANCE_DEG) {
            /* MATCH FOUND */
            FusedObject fused;
            fused.object_type    = pending_camera[i].object_type;
            fused.distance_m     = ld->distance_m;
            fused.angle_deg      = (ld->angle_deg + pending_camera[i].angle_deg) / 2.0f;
            fused.size           = pending_camera[i].size;
            fused.lidar_id       = ld->object_id;
            fused.camera_id      = pending_camera[i].object_id;
            fused.confidence_pct = compute_confidence(ld, &pending_camera[i]);

            log_event("[FUSION] Matched: LiDAR_ID=%u (%.1fdeg) <-> Camera_ID=%u (%.1fdeg) diff=%.1fdeg\n",
                      ld->object_id, ld->angle_deg,
                      pending_camera[i].object_id, pending_camera[i].angle_deg,
                      diff);

            print_fused_object(&fused);
            remove_camera_pending(i);
            return 1;
        }
    }

    /* No match — store in pending LiDAR buffer */
    if (pending_lidar_count < MAX_PENDING) {
        pending_lidar[pending_lidar_count++] = *ld;
        log_event("[FUSION] No camera match for LiDAR_ID=%u (angle=%.1fdeg). Pending LiDAR count: %d\n",
                  ld->object_id, ld->angle_deg, pending_lidar_count);
    } else {
        log_event("[FUSION] Warning: pending LiDAR buffer full — dropping LiDAR_ID=%u\n",
                  ld->object_id);
    }
    return 0;
}

/* =========================================================
 * try_match_camera()
 * =========================================================
 * A new Camera detection has arrived. Scan the pending LiDAR
 * array for a match.
 *
 * Logic mirrors try_match_lidar().
 */
static int try_match_camera(const CameraDetection *cd)
{
    for (int i = 0; i < pending_lidar_count; i++) {
        float diff = fabsf(cd->angle_deg - pending_lidar[i].angle_deg);
        if (diff <= ANGLE_TOLERANCE_DEG) {
            /* MATCH FOUND */
            FusedObject fused;
            fused.object_type    = cd->object_type;
            fused.distance_m     = pending_lidar[i].distance_m;
            fused.angle_deg      = (pending_lidar[i].angle_deg + cd->angle_deg) / 2.0f;
            fused.size           = cd->size;
            fused.lidar_id       = pending_lidar[i].object_id;
            fused.camera_id      = cd->object_id;
            fused.confidence_pct = compute_confidence(&pending_lidar[i], cd);

            log_event("[FUSION] Matched: Camera_ID=%u (%.1fdeg) <-> LiDAR_ID=%u (%.1fdeg) diff=%.1fdeg\n",
                      cd->object_id, cd->angle_deg,
                      pending_lidar[i].object_id, pending_lidar[i].angle_deg,
                      diff);

            print_fused_object(&fused);
            remove_lidar_pending(i);
            return 1;
        }
    }

    /* No match — store in pending camera buffer */
    if (pending_camera_count < MAX_PENDING) {
        pending_camera[pending_camera_count++] = *cd;
        log_event("[FUSION] No LiDAR match for Camera_ID=%u (angle=%.1fdeg). Pending Camera count: %d\n",
                  cd->object_id, cd->angle_deg, pending_camera_count);
    } else {
        log_event("[FUSION] Warning: pending camera buffer full — dropping Camera_ID=%u\n",
                  cd->object_id);
    }
    return 0;
}

/* =========================================================
 * discard_stale()
 * =========================================================
 * Scans both pending arrays.
 * Any detection older than STALE_THRESHOLD_MS is:
 *   1. Reported as a single-sensor detection.
 *   2. Logged as stale.
 *   3. Removed from the pending array.
 *
 * We iterate backwards so that remove_*_pending() index
 * shifts do not cause us to skip entries.
 */
static void discard_stale(void)
{
    /* Check pending LiDAR detections */
    for (int i = pending_lidar_count - 1; i >= 0; i--) {
        uint64_t age = lidar_age_ms(&pending_lidar[i]);
        if (age > STALE_THRESHOLD_MS) {
            log_event("[FUSION] Discarding stale LiDAR detection: ID=%u age=%llu ms\n",
                      pending_lidar[i].object_id, (unsigned long long)age);
            print_lidar_only(&pending_lidar[i]);
            remove_lidar_pending(i);
        }
    }

    /* Check pending Camera detections */
    for (int i = pending_camera_count - 1; i >= 0; i--) {
        uint64_t age = camera_age_ms(&pending_camera[i]);
        if (age > STALE_THRESHOLD_MS) {
            log_event("[FUSION] Discarding stale Camera detection: ID=%u age=%llu ms\n",
                      pending_camera[i].object_id, (unsigned long long)age);
            print_camera_only(&pending_camera[i]);
            remove_camera_pending(i);
        }
    }
}

/* =========================================================
 * read_lidar_fifo()
 * =========================================================
 * Reads one LidarDetection struct from the FIFO.
 * Handles partial reads (loops until full struct read or error).
 *
 * Returns  1 if a complete detection was read.
 * Returns  0 if the FIFO would block (no data yet — EAGAIN/EWOULDBLOCK).
 * Returns -1 on EOF or fatal error.
 */
static int read_lidar_fifo(void)
{
    LidarDetection det;
    uint8_t *buf = (uint8_t *)&det;
    size_t   total = sizeof(LidarDetection);
    size_t   received = 0;

    while (received < total) {
        ssize_t n = read(g_lidar_fd, buf + received, total - received);
        if (n > 0) {
            received += (size_t)n;
        } else if (n == 0) {
            /* EOF: writer closed the FIFO */
            /* Reopen in non-blocking mode to wait for the next writer */
            log_event("[FUSION] LiDAR FIFO writer disconnected (EOF). Reopening...\n");
            close(g_lidar_fd);
            g_lidar_fd = open(LIDAR_FIFO_PATH, O_RDONLY | O_NONBLOCK);
            if (g_lidar_fd < 0) {
                perror("[FUSION] Failed to reopen lidar_fifo");
                return -1;
            }
            return 0;
        } else {
            /* n < 0 */
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* No data available right now */
                if (received == 0) return 0;   /* Clean no-data */
                /* Partial read — shouldn't happen with a well-behaved writer,
                   but handle gracefully by discarding the partial struct */
                log_event("[FUSION] Warning: partial LiDAR read (%zu/%zu bytes) — discarding.\n",
                          received, total);
                return 0;
            } else if (errno == EINTR) {
                /* Interrupted by signal — retry */
                continue;
            } else {
                perror("[FUSION] read() on lidar_fifo failed");
                return -1;
            }
        }
    }

    /* Validate basic fields */
    if (det.distance_m < 0.0f || det.distance_m > 1000.0f) {
        log_event("[FUSION] Warning: invalid LiDAR distance %.1f — discarding.\n",
                  det.distance_m);
        return 0;
    }
    if (det.angle_deg < -360.0f || det.angle_deg > 360.0f) {
        log_event("[FUSION] Warning: invalid LiDAR angle %.1f — discarding.\n",
                  det.angle_deg);
        return 0;
    }

    log_event("[FUSION] Received LiDAR detection: ID=%u Dist=%.1fm Angle=%.1fdeg\n",
              det.object_id, det.distance_m, det.angle_deg);

    try_match_lidar(&det);
    return 1;
}

/* =========================================================
 * read_camera_fifo()
 * =========================================================
 * Reads one CameraDetection struct from the FIFO.
 * Same logic as read_lidar_fifo().
 */
static int read_camera_fifo(void)
{
    CameraDetection det;
    uint8_t *buf = (uint8_t *)&det;
    size_t   total = sizeof(CameraDetection);
    size_t   received = 0;

    while (received < total) {
        ssize_t n = read(g_camera_fd, buf + received, total - received);
        if (n > 0) {
            received += (size_t)n;
        } else if (n == 0) {
            log_event("[FUSION] Camera FIFO writer disconnected (EOF). Reopening...\n");
            close(g_camera_fd);
            g_camera_fd = open(CAMERA_FIFO_PATH, O_RDONLY | O_NONBLOCK);
            if (g_camera_fd < 0) {
                perror("[FUSION] Failed to reopen camera_fifo");
                return -1;
            }
            return 0;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (received == 0) return 0;
                log_event("[FUSION] Warning: partial Camera read (%zu/%zu bytes) — discarding.\n",
                          received, total);
                return 0;
            } else if (errno == EINTR) {
                continue;
            } else {
                perror("[FUSION] read() on camera_fifo failed");
                return -1;
            }
        }
    }

    /* Validate */
    if (det.object_type < OBJ_UNKNOWN || det.object_type > OBJ_ANIMAL) {
        log_event("[FUSION] Warning: invalid Camera object_type %d — discarding.\n",
                  (int)det.object_type);
        return 0;
    }
    if (det.angle_deg < -360.0f || det.angle_deg > 360.0f) {
        log_event("[FUSION] Warning: invalid Camera angle %.1f — discarding.\n",
                  det.angle_deg);
        return 0;
    }

    log_event("[FUSION] Received Camera detection: ID=%u Type=%s Angle=%.1fdeg Size=%s\n",
              det.object_id,
              OBJECT_TYPE_LABELS[det.object_type],
              det.angle_deg,
              OBJECT_SIZE_LABELS[det.size]);

    try_match_camera(&det);
    return 1;
}

/* =========================================================
 * cleanup()
 * =========================================================
 * Close file descriptors and log file.
 * The FIFOs themselves are left on the filesystem so they
 * can be reused across runs. To remove them: make clean.
 */
static void cleanup(void)
{
    if (g_lidar_fd >= 0) {
        close(g_lidar_fd);
        g_lidar_fd = -1;
    }
    if (g_camera_fd >= 0) {
        close(g_camera_fd);
        g_camera_fd = -1;
    }
    if (g_log_fp) {
        fprintf(g_log_fp,
                "============================================================\n"
                "  Fusion Process Stopped\n"
                "============================================================\n\n");
        fclose(g_log_fp);
        g_log_fp = NULL;
    }
}

/* =========================================================
 * main()
 * ========================================================= */
int main(void)
{
    printf("=====================================================\n");
    printf("   Multi-Sensor Fusion Process\n");
    printf("   Angle Tolerance : %.1f degrees\n", ANGLE_TOLERANCE_DEG);
    printf("   Stale Threshold : %d ms\n",        STALE_THRESHOLD_MS);
    printf("=====================================================\n\n");

    /* Set up signal handlers */
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    /* Ignore SIGPIPE at process level — handle via read() errno */
    signal(SIGPIPE, SIG_IGN);

    /* Open log file */
    open_log();
    log_event("[FUSION] Process started. PID=%d\n", (int)getpid());

    /* Create FIFOs */
    if (create_fifos() < 0) {
        cleanup();
        return EXIT_FAILURE;
    }

    /* Open FIFOs in non-blocking mode */
    if (open_fifos_nonblocking() < 0) {
        cleanup();
        return EXIT_FAILURE;
    }

    log_event("[FUSION] Listening on lidar_fifo and camera_fifo...\n\n");

    /* --------------------------------------------------
     * Main select() loop
     *
     * select() monitors both FIFOs simultaneously.
     * Timeout is set to 500 ms so that stale detection
     * checks run at least every 500 ms even if no sensor
     * data arrives.
     * -------------------------------------------------- */
    while (g_running) {

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(g_lidar_fd,  &read_fds);
        FD_SET(g_camera_fd, &read_fds);

        int nfds = (g_lidar_fd > g_camera_fd ? g_lidar_fd : g_camera_fd) + 1;

        /* Timeout: 500 ms */
        struct timeval timeout;
        timeout.tv_sec  = 0;
        timeout.tv_usec = 500 * 1000;

        int ret = select(nfds, &read_fds, NULL, NULL, &timeout);

        if (ret < 0) {
            if (errno == EINTR) {
                /* Interrupted by signal (e.g. Ctrl+C) — let loop condition handle it */
                continue;
            }
            perror("[FUSION] select() failed");
            break;
        }

        /* ret == 0 → timeout (no data) — fall through to stale check */

        /* Check if LiDAR FIFO has data */
        if (ret > 0 && FD_ISSET(g_lidar_fd, &read_fds)) {
            if (read_lidar_fifo() < 0) {
                log_event("[FUSION] Fatal error reading LiDAR FIFO. Stopping.\n");
                break;
            }
        }

        /* Check if Camera FIFO has data */
        if (ret > 0 && FD_ISSET(g_camera_fd, &read_fds)) {
            if (read_camera_fifo() < 0) {
                log_event("[FUSION] Fatal error reading Camera FIFO. Stopping.\n");
                break;
            }
        }

        /* Always run stale-detection check after every select() cycle */
        discard_stale();
    }

    /* --------------------------------------------------
     * Report any remaining pending detections on shutdown
     * -------------------------------------------------- */
    fprintf(stderr, "\n[FUSION] Caught shutdown signal. Cleaning up...\n");
    log_event("[FUSION] Shutdown: reporting remaining pending detections...\n");

    for (int i = 0; i < pending_lidar_count; i++) {
        print_lidar_only(&pending_lidar[i]);
    }
    for (int i = 0; i < pending_camera_count; i++) {
        print_camera_only(&pending_camera[i]);
    }

    cleanup();
    printf("[FUSION] Process stopped cleanly.\n");
    return EXIT_SUCCESS;
}
