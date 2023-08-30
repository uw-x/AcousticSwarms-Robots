#ifndef __NAV_H__
#define __NAV_H__

#include <stdint.h>
#include <stdbool.h>
#include "point.h"
#include "recorder.h"

typedef struct {
  float mtr_val;
  
  point position;
  point velocity;

  float angular_velocity, pitch_velocity;
  float rotation, pitch;
  float unwrapped_rotation;
} nav_state_t;

typedef enum {
  NAV_ACTION_IDLE = 1,
  NAV_ACTION_FORWARD,
  NAV_ACTION_ROTATE,
  NAV_ACTION_BACKWARD,
  NAV_ACTION_RESPONSIVE, // Perform sensor updates, but don't do anything
  NAV_ACTION_ROTATE_PRECISE,
} action_type_t;

typedef struct {
  point target_position;
  float motion_correction_begin_time;
} action_forward_data_t;

typedef struct {
  float target_rotation;
} action_rotate_data_t;

typedef enum {
  NAV_TARGET_STATE = 0,
  NAV_TARGET_TIME,
} action_target_t;

typedef struct {
  action_type_t type;
  int64_t start_time;
  bool notify_when_done;
  action_target_t target;
  bool controlled;
  float speed;
  union {
    action_forward_data_t forward_data;
    action_rotate_data_t rotation_data;
  } params;
  uint64_t duration_us;
} action_t;

typedef enum {
  MOTION_FORWARD = 0,
  MOTION_BACKWARD,
  MOTION_ROTATE_ANGLE,
  MOTION_ROTATE_TIME,
  MOTION_BRAKE,
  MOTION_FORWARD_PULSE, 
  MOTION_BACKWARD_PULSE,
  MOTION_ROTATE_ANGLE_PRECISE,
} motion_code_t;

typedef  enum{
  NAV_FLAG_FORCE_UPDATE = 0,
  NAV_FLAG_UPDATE_ANGLE = 1,
} motion_flags; 


void nav_init(void);
void nav_begin(void);
void nav_end(void);
void nav_stop(void);

void nav_reset_beta(void);

void nav_clear_action_queue(void);
void nav_motion_update(motion_code_t code, uint8_t const * const args, uint8_t length);
void nav_position_update(nav_state_t *estimated_state, nav_state_t *current_milestone, nav_state_t *next_milestone, uint8_t flags);

bool nav_xcorr_stream_enabled(void);
void nav_enable_xcorr_stream(bool use_notifications);
void nav_disable_xcorr_stream(void);
nav_state_t nav_get_current_state(void);
nav_state_t nav_get_previous_state_estimate(void);
action_t nav_get_current_action(void);
void nav_set_current_action(action_t new_action);
void nav_set_rotation_threshold(float thresh);
void nav_reset_rotation_threshold(void);

void nav_send_all(uint16_t sample_offset, recording_t *r);

void nav_add_milestone(point *milestone);
void nav_clear_milestones(void);
bool nav_add_action(action_t *action);
void nav_acc_update(void);
void nav_gyro_update(void);
void nav_recover(void);

void nav_forward(uint64_t duration_us, int16_t speed, bool controlled);
void nav_backward(uint64_t duration_us, int16_t speed, bool controlled);
void nav_rotate_angle(float angle);
void nav_rotate_angle_precise(float angle);
void nav_rotate_time(uint64_t duration_us, int16_t speed);

void nav_update_pitch(void);

void nav_disable_collision_detection(void);
void nav_enable_collision_detection(void);

#endif