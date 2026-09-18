/*
 * common.h
 * --------
 * Shared definitions for the Multi-Sensor Fusion Pipeline project.
 *
 * This header is included by all three programs:
 *   - lidar_simulator.c
 *   - camera_simulator.c
 *   - fusion.c
 *
 * It defines:
 *   - FIFO paths
 *   - Tunable constants
 *   - Data structures for LiDAR, Camera, and Fused objects
 *   - Object type labels (for camera)
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <time.h>

/* =========================================================
 * FIFO (Named Pipe) Paths
 * =========================================================
 * These files will be created on the Linux filesystem.
 * Both sensor simulators write to their own FIFO.
 * The Fusion process reads from both.
 */
#define LIDAR_FIFO_PATH   "lidar_fifo"
#define CAMERA_FIFO_PATH  "camera_fifo"

/* =========================================================
 * Simulation Timing
 * =========================================================
 * How often each sensor sends a detection (milliseconds).
 */
#define LIDAR_INTERVAL_MS   800    /* LiDAR sends every 800 ms  */
#define CAMERA_INTERVAL_MS  600    /* Camera sends every 600 ms */

/* =========================================================
 * Sensor Association / Fusion Parameters
 * =========================================================
 * ANGLE_TOLERANCE_DEG:
 *   Two detections are considered the same physical object
 *   if their angle difference is <= this value.
 *
 * STALE_THRESHOLD_MS:
 *   A detection older than this (in milliseconds) is discarded
 *   even if it was never matched.
 *
 * MAX_PENDING:
 *   Maximum number of unmatched detections held in memory
 *   for each sensor at any given time.
 */
#define ANGLE_TOLERANCE_DEG  5.0f
#define STALE_THRESHOLD_MS   2000
#define MAX_PENDING          16

/* =========================================================
 * Logging
 * =========================================================
 */
#define LOG_FILE_PATH  "logs/fusion.log"

/* =========================================================
 * Object Types (Camera Labels)
 * =========================================================
 * The camera classifier assigns one of these integer codes.
 * The fusion output prints the human-readable string.
 */
typedef enum {
    OBJ_UNKNOWN  = 0,
    OBJ_PERSON   = 1,
    OBJ_CAR      = 2,
    OBJ_TRUCK    = 3,
    OBJ_BICYCLE  = 4,
    OBJ_ANIMAL   = 5
} ObjectType;

/* String labels matching the ObjectType enum (index == enum value) */
static const char *OBJECT_TYPE_LABELS[] = {
    "Unknown",
    "Person",
    "Car",
    "Truck",
    "Bicycle",
    "Animal"
};

/* =========================================================
 * Object Size Labels (Camera)
 * =========================================================
 */
typedef enum {
    SIZE_SMALL  = 0,
    SIZE_MEDIUM = 1,
    SIZE_LARGE  = 2
} ObjectSize;

static const char *OBJECT_SIZE_LABELS[] = {
    "Small",
    "Medium",
    "Large"
};

/* =========================================================
 * LiDAR Detection Structure
 * =========================================================
 * Written by lidar_simulator.c → lidar_fifo
 * Read by fusion.c
 *
 * Fields:
 *   object_id  - Integer ID assigned by the LiDAR simulator
 *   distance_m - Distance to the detected object in metres
 *   angle_deg  - Azimuth angle in degrees (0 = straight ahead)
 *   timestamp  - Unix time (seconds since epoch) when detected
 *   timestamp_ms - Millisecond part of the timestamp
 */
typedef struct {
    uint32_t object_id;
    float    distance_m;
    float    angle_deg;
    time_t   timestamp;
    uint32_t timestamp_ms;
} LidarDetection;

/* =========================================================
 * Camera Detection Structure
 * =========================================================
 * Written by camera_simulator.c → camera_fifo
 * Read by fusion.c
 *
 * Fields:
 *   object_id    - Integer ID assigned by the Camera simulator
 *   object_type  - Classified object type (see ObjectType enum)
 *   angle_deg    - Angle in degrees to the detected object
 *   size         - Approximate object size (see ObjectSize enum)
 *   timestamp    - Unix time when detected
 *   timestamp_ms - Millisecond part of the timestamp
 */
typedef struct {
    uint32_t   object_id;
    ObjectType object_type;
    float      angle_deg;
    ObjectSize size;
    time_t     timestamp;
    uint32_t   timestamp_ms;
} CameraDetection;

/* =========================================================
 * Fused Object Structure
 * =========================================================
 * Produced internally by fusion.c when a LiDAR detection
 * and a Camera detection are matched.
 *
 * Fields:
 *   object_type     - From camera classification
 *   distance_m      - From LiDAR measurement
 *   angle_deg       - Average of LiDAR and Camera angles
 *   size            - From camera
 *   confidence_pct  - Rule-based confidence score (0–100)
 *   lidar_id        - Source LiDAR object_id
 *   camera_id       - Source Camera object_id
 *
 * Confidence Calculation (rule-based, simple):
 *   Base confidence = 70%
 *   If angle difference < 1.0 deg : +15%  (very close match)
 *   If angle difference < 3.0 deg : +10%  (good match)
 *   If angle difference < 5.0 deg : +5%   (acceptable match)
 *   If camera object type != UNKNOWN : +10%
 *   If camera size != SMALL        : +5%  (larger objects easier to see)
 *   Maximum capped at 99%.
 */
typedef struct {
    ObjectType object_type;
    float      distance_m;
    float      angle_deg;
    ObjectSize size;
    int        confidence_pct;
    uint32_t   lidar_id;
    uint32_t   camera_id;
} FusedObject;

/* =========================================================
 * Utility: Get current time in milliseconds since epoch
 * =========================================================
 * Used by all three programs for timestamping detections.
 */
#include <sys/time.h>
static inline uint64_t get_time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000ULL + (uint64_t)(tv.tv_usec) / 1000ULL;
}

/* Populate timestamp fields of a LidarDetection from the current time */
static inline void lidar_stamp(LidarDetection *d)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    d->timestamp    = tv.tv_sec;
    d->timestamp_ms = (uint32_t)(tv.tv_usec / 1000);
}

/* Populate timestamp fields of a CameraDetection from the current time */
static inline void camera_stamp(CameraDetection *d)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    d->timestamp    = tv.tv_sec;
    d->timestamp_ms = (uint32_t)(tv.tv_usec / 1000);
}

/* Return the age of a LidarDetection in milliseconds */
static inline uint64_t lidar_age_ms(const LidarDetection *d)
{
    uint64_t now = get_time_ms();
    uint64_t det = (uint64_t)(d->timestamp) * 1000ULL + d->timestamp_ms;
    return (now > det) ? (now - det) : 0;
}

/* Return the age of a CameraDetection in milliseconds */
static inline uint64_t camera_age_ms(const CameraDetection *d)
{
    uint64_t now = get_time_ms();
    uint64_t det = (uint64_t)(d->timestamp) * 1000ULL + d->timestamp_ms;
    return (now > det) ? (now - det) : 0;
}

#endif /* COMMON_H */
