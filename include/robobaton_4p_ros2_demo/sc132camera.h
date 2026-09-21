#ifndef SC132_CAMERA_H
#define SC132_CAMERA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SC132_ABI_VERSION_MAJOR 2U
#define SC132_ABI_VERSION_MINOR 0U

#define SC132_STATUS_OK ((int32_t)0)
#define SC132_STATUS_INVALID_ARGUMENT ((int32_t)-1)
#define SC132_STATUS_INVALID_STATE ((int32_t)-2)
#define SC132_STATUS_STARTUP_FAILED ((int32_t)-3)

#define SC132_FRAME_SET_MAX_CAMERAS 4U
#define SC132_NATIVE_OUTPUT_WIDTH 1280U
#define SC132_NATIVE_OUTPUT_HEIGHT 1088U
#define SC132_FRAME_SET_DEFAULT_MAX_SKEW_NS 10000000ULL

/*
 * The frame-set output canvas is native 1280x1088 (sensor-axis 1088x1280 with internal
 * 0/180-degree rotation). The public VSE full-frame scaling canvases are 640x480,
 * 720x480, and 1280x720; either axis notation is valid (external 90/270-degree
 * rotation swaps the delivered width and height). Scaling does not change the FOV:
 * the full frame is stretched to the target size, changing the aspect ratio relative
 * to the native canvas; software rotation (180/270 degrees) is performed by Nano2D
 * after scaling.
 */

typedef struct sc132_frame sc132_frame_t;

typedef struct sc132_frame_info {
  uint32_t struct_size;
  uint32_t camera_id;
  uint64_t sequence;
  uint32_t frame_id;
  uint32_t reserved0;
  uint64_t timestamp_ns;
  const void *y_data;
  const void *uv_data;
  uint64_t y_phys;
  uint64_t uv_phys;
  uint64_t y_size;
  uint64_t uv_size;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t vstride;
  uint32_t reserved[8];
} sc132_frame_info_t;

typedef struct sc132_frame_set_item {
  sc132_frame_t *frame;
  uint32_t camera_id;
  uint32_t frame_id;
  uint64_t sequence;
  uint64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
} sc132_frame_set_item_t;

typedef struct sc132_frame_set {
  uint32_t struct_size;
  uint32_t camera_count;
  uint64_t group_id;
  uint64_t group_timestamp_ns;
  uint64_t max_skew_ns;
  sc132_frame_set_item_t items[SC132_FRAME_SET_MAX_CAMERAS];
  uint32_t reserved[8];
} sc132_frame_set_t;

/*
 * DMA frames are read-only. The contract below defines the time domain, borrowed
 * references, and cross-callback retention rules. timestamp_ns is in ns. In
 * software_gpio mode, a frame set uses the CLOCK_MONOTONIC_RAW rising-edge time
 * from GPIO417, identical for all cameras in the set. Other modes prefer the
 * per-frame sensor/VIO time and fall back to the system output-frame time when it
 * is unavailable. No mode guarantees the wall-clock domain.
 * The frame_set and item array are valid only during the callback; items[i].frame
 * is a borrowed reference. Consumers must not call sc132_frame_release directly
 * on a callback reference owned by the library. To retain a frame across callbacks,
 * call sc132_frame_retain in the callback and call sc132_frame_release after the
 * final use. user_data must remain valid until blocking sc132_stop returns; a
 * request_stop return does not mean that callbacks have exited. C++ consumer
 * callbacks must be noexcept and catch all exceptions internally; exceptions must
 * not cross this C ABI. The Y/UV addresses in frame_info are read-only and valid
 * only while the corresponding frame reference remains valid.
 */
typedef void (*sc132_frame_set_callback_t)(const sc132_frame_set_t *frame_set,
                                            void *user_data);

typedef struct sc132_frame_set_config {
  uint32_t struct_size;
  sc132_frame_set_callback_t callback;
  void *user_data;
  uint32_t camera_count;
  uint32_t width;
  uint32_t height;
  uint32_t timeout_ms;
  uint64_t max_skew_ns;
  uint32_t reserved[8];
} sc132_frame_set_config_t;

#define SC132_FRAME_SET_CONFIG_INIT \
  { sizeof(sc132_frame_set_config_t), NULL, NULL, 4U, SC132_NATIVE_OUTPUT_WIDTH, SC132_NATIVE_OUTPUT_HEIGHT, 100U, SC132_FRAME_SET_DEFAULT_MAX_SKEW_NS, {0U} }

/* Product release SemVer; returned storage is process-static and read-only. */
const char *sc132_get_version(void);
int32_t sc132_set_fps(uint32_t fps);
int32_t sc132_set_output_rotation(uint32_t rotate_clockwise_degrees);
int32_t sc132_start_frame_set(const sc132_frame_set_config_t *config,
                              uint32_t camera_mask);
int32_t sc132_frame_retain(sc132_frame_t *frame);
void sc132_frame_release(sc132_frame_t *frame);
int32_t sc132_frame_get_info(const sc132_frame_t *frame,
                             sc132_frame_info_t *out_info);
/*
 * Two-phase shutdown prevents a blocking callback from waiting on a retained frame.
 * request_stop linearizes shutdown, rejects new acquisition and callback admission,
 * and wakes waiters. It does not drain or release frames, call vendor APIs, join
 * trigger/worker/dispatcher threads, or wait for callbacks, so it can return quickly
 * from any thread. Calling request_stop while idle also latches STOPPING; a non-
 * callback thread must then call blocking sc132_stop to complete that stop generation
 * before start/config becomes available again.
 * The normal lifecycle owner then calls blocking sc132_stop. It drains pending and
 * queued frames, waits for in-flight callbacks, joins library-created threads, and
 * closes I2C. If a vendor worker never exits, sc132_stop may remain blocked and the
 * lifecycle remains STOPPING until actual quiescence; it does not report STOPPED early.
 * In the rare event of an internal pthread_join failure, sc132_stop retains thread
 * ownership, I2C, and callback/config state while releasing the cleanup owner but
 * remaining STOPPING. An external non-callback thread must retry sc132_stop; start
 * and unload remain forbidden until it succeeds.
 * After sc132_start_frame_set returns STARTUP_FAILED, an external non-callback thread
 * must call sc132_stop to reach quiescence before unloading this library. Until then,
 * later start/config calls return INVALID_STATE, and the library creates no detached
 * cleanup guard. If stop is called from the dispatcher callback thread, it only issues
 * request_stop and returns; an external non-callback thread must call stop again to
 * perform the sole cleanup owner's drain/join and avoid deterministic self-deadlock.
 */
void sc132_request_stop(void);
void sc132_stop(void);

#ifdef __cplusplus
}
#endif

#endif
