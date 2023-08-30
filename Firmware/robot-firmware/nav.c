#include "nav.h"
#include "motors.h"
#include "point.h"
#include "math.h"
#include "gyro.h"
#include "timers.h"
#include "accel.h"
#include "event.h"
#include "led.h"
#include "ble_manager.h"
#include "ble_dcs.h"
#include "ir.h"
#include "nrf_log.h"

#define LED_MARKER_ENABLED 1
#define LED_FIXED_COLOR led_rgb(1,1,1)
#define USE_LED_FIXED_COLOR 0
#define USE_PITCH_COMPENSATION 1

#define GYRO_CONTROL_ENABLE 1
#define POS_THRESHOLD 2
#define ROTATION_THRESHOLD_DEFAULT 6
#define MOTION_CORRECTION_ANGLE_THRESHOLD 90
#define NAV_FORWARD_SPEED_INITIAL 200
#define NAV_ROTATE_SPEED_INITIAL 200

#define ADD_MILESTONE_DEBUG 1
#define ESTIMATED_POSITION_DEBUG 0
#define ESTIMATED_ROTATION_DEBUG 0
#define POSITION_UPDATES_DEBUG 0
#define YAW_CONTROL_DEBUG 0
#define NAV_ROTATE_DEBUG 0
#define DISTANCE_DEBUG 0

#define INITIAL_POSITION_X 0
#define INITIAL_POSITION_Y 0
#define INITIAL_ROTATION 0

static bool xcorr_stream_enabled = false;
static bool xcorr_stream_use_notifications;
static float m_rotation_thresh = ROTATION_THRESHOLD_DEFAULT;
static bool nav_collision_detection_enabled = false;
static nav_state_t last_state_estimate;
static nav_state_t current_state;
static nav_state_t target_state;

// Estimated inter-wheel coefficients
static float beta = 0.5;

static uint64_t previous_system_time;
static float previous_yaw_rate;

static uint64_t current_system_time;
static float current_yaw_rate;

static bool m_debug_flag = false;

#define NUM_ACTIONS_MAX 20
static uint8_t m_action_queue_head = 0;
static uint8_t m_action_queue_tail = 0;
static action_t m_action_queue[NUM_ACTIONS_MAX];
static action_t current_action;

#define NUM_MILESTONES_MAX 125
static point milestones[NUM_MILESTONES_MAX];
static int8_t m_current_milestone_idx;
static int8_t num_milestones;

static int64_t gyro_previous_time = 0;
static int64_t acc_previous_time = 0;
static int64_t pos_previous_time = 0;
static int64_t nav_update_previous_time;

const static action_t IDLE = {.type = NAV_ACTION_IDLE};

#define W_PID_KP 0.25
#define W_PID_KI 0
#define W_PID_KD 0

#define F_PID_KP 0.9
#define F_PID_KI 0.1
#define F_PID_KD 0

#define R_PID_KP 0.7
#define R_PID_KI 0
#define R_PID_KD 0

typedef struct{
  float kp, ki, kd;
  float x, sx;
} pid_t;

static pid_t stability_controller, rotation_controller, forward_controller;
static volatile bool m_nav_enabled;

void pid_init(pid_t *p, float kp, float ki, float kd){
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;

  p->x = 0;
  p->sx = 0;
}

// Updates PID state with new error value and returns PID output
float pid_put_next(pid_t *p, float err_sig, float dt){
  float ret = err_sig * p->kp + p->sx * p->ki + (err_sig - p->x) / dt * p->kd;
#if YAW_CONTROL_DEBUG
  NRF_LOG_RAW_INFO("[pid] err_sig " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(err_sig * p->kp));
  NRF_LOG_RAW_INFO("[pid] I " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(p->sx * p->ki));
  NRF_LOG_RAW_INFO("[pid] D " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT((err_sig - p->x) / dt * p->kd));
  NRF_LOG_RAW_INFO("[pid] ret " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(ret));
#endif
  p->x = err_sig;
  p->sx += err_sig * dt;
  return ret;
}

bool nav_xcorr_stream_enabled(){
  return xcorr_stream_enabled;
}

void nav_enable_xcorr_stream(bool use_notifications){
  xcorr_stream_enabled = true;
  xcorr_stream_use_notifications = use_notifications;
}
void nav_disable_xcorr_stream(void){
  xcorr_stream_enabled = false;
}

//#pragma GCC push_options
//#pragma GCC optimize ("O0")
bool nav_add_action(action_t *action){
  if((m_action_queue_head + NUM_ACTIONS_MAX - m_action_queue_tail) % NUM_ACTIONS_MAX  == NUM_ACTIONS_MAX - 1){
    return false;
  }
  m_action_queue[m_action_queue_head] = *action;
  NRF_LOG_INFO("Notify when done: %d", action->notify_when_done);
  NRF_LOG_INFO("Notify when done: %d", m_action_queue[m_action_queue_head].notify_when_done);
  m_action_queue_head = (m_action_queue_head + 1) % NUM_ACTIONS_MAX;
  NRF_LOG_INFO("Added action: %d", action->type);
  

  return true;
}

void nav_set_current_action(action_t new_action){
  //if(new_action.type != current_action.type){
  //  //NRF_LOG_INFO("Before %d \tAfter: %d", current_action.type, new_action.type);
  //  ble_indicate_transition(current_action.type, new_action.type);
  //}
  current_action = new_action;
}

void nav_add_milestone(point *milestone){
  milestones[num_milestones] = *milestone;
#if ADD_MILESTONE_DEBUG
  NRF_LOG_RAW_INFO("[nav] Added milestone: ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n",
  NRF_LOG_FLOAT(milestones[num_milestones].x),
  NRF_LOG_FLOAT(milestones[num_milestones].y));
#endif
  num_milestones++;
}

void _print_state(void){
  NRF_LOG_RAW_INFO("Current rotation: " NRF_LOG_FLOAT_MARKER,
                    NRF_LOG_FLOAT(current_state.rotation));
  NRF_LOG_RAW_INFO(" Target rotation: " NRF_LOG_FLOAT_MARKER "\n",
                    NRF_LOG_FLOAT(target_state.rotation));
}

action_t get_idle_action(void){
  action_t act = IDLE;
  act.start_time = systemTimeGetUs();
  return act;
}

void nav_clear_action_queue(void){
  current_action = get_idle_action();
  m_action_queue_head = m_action_queue_tail = 0;
  motors_hard_brake();
}

bool nav_process_next_action(void){
  if(current_action.type != NAV_ACTION_IDLE){
    motors_brake();
  }

  // If no actions left in the queue, set action to IDLE.
  if(m_action_queue_tail == m_action_queue_head){
    nav_set_current_action(get_idle_action());    
    return false;
  }

  action_t action = m_action_queue[m_action_queue_tail];
  m_action_queue_tail = (m_action_queue_tail + 1) % NUM_ACTIONS_MAX;

  nav_set_current_action(action);

  //NRF_LOG_RAW_INFO("[nav] Current action: %d\n", current_action.type);
  current_state.velocity.x = current_state.velocity.y = 0;
  current_state.angular_velocity = 0;

  // Do action
  switch(current_action.type){
    case NAV_ACTION_FORWARD:
      current_state.mtr_val = current_action.speed;
      //target_state.position = current_action.params.forward_data.target_position;
      target_state.rotation = current_state.rotation;
      
      point dir = point_sub(target_state.position, current_state.position);
      float distance = point_mag(&dir);
      
      // t = 0.35 / v * d
      //current_action.params.forward_data.motion_correction_begin_time = (0.35/5) * distance;
      current_action.params.forward_data.motion_correction_begin_time = 1;
      
      pid_init(&stability_controller, W_PID_KP, W_PID_KI, W_PID_KD);
      pid_init(&forward_controller, F_PID_KP, F_PID_KI, F_PID_KD);
      
      //accel_update_biases();
      //gyro_update_biases();
      //gyro_reset_biases();
      //accel_reset_biases();
      motors_adjust(current_state.mtr_val, beta);
    break;
    case NAV_ACTION_ROTATE:
      if(current_action.target == NAV_TARGET_STATE){
        target_state.rotation = current_action.params.rotation_data.target_rotation;
        pid_init(&rotation_controller, R_PID_KP, R_PID_KI, R_PID_KD);
      }
    break;
    case NAV_ACTION_ROTATE_PRECISE:
    target_state.rotation = current_action.params.rotation_data.target_rotation;
    break;
    case NAV_ACTION_BACKWARD:
      motors_adjust(-current_action.speed, beta);
      //motors_backward(200);
    break;
    case NAV_ACTION_RESPONSIVE:
    break;
    default:
    break;
  }

  // Reset time
  int64_t time = systemTimeGetUs();
  gyro_previous_time = time;
  acc_previous_time = time;
  pos_previous_time = time;
  nav_update_previous_time = time;
  current_action.start_time = time;

  return true;
}

void nav_brake(void){
  motors_brake();
}

float mod_abs(float a1, float a2, float mod){
  float a = fmod(a1 + (mod - a2) , mod) ;
  float b = fmod(a2 + (mod - a1) , mod) ;
  return a < b ? a : b;
}

void nav_process_next_milestone(void){
    static char buff[256];
    uint8_t msg_length = sprintf(buff, "milestone_idx: %d / %d", m_current_milestone_idx, num_milestones);
    ble_indicate_status(DCS_STATUS_LOG, buff, msg_length);
    if(m_current_milestone_idx < num_milestones){
      target_state.position = milestones[m_current_milestone_idx];
      
      point dir = point_sub(target_state.position, current_state.position);
      float arg = point_arg(&dir) * 180 / M_PI;
      if(arg < 0) arg += 360;
      float angle_distance = fmod(arg + 360 - current_state.rotation, 360);
      if(angle_distance > 180) angle_distance = 360 - angle_distance;

      float mag = point_mag(&dir);

      NRF_LOG_RAW_INFO("[nav] Angle: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(angle_distance));
      NRF_LOG_RAW_INFO("[nav] Magnitude: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(mag));

      if(fabs(angle_distance) > m_rotation_thresh ){
        action_t rotation_act = {
          .type = NAV_ACTION_ROTATE,
          .target = NAV_TARGET_STATE,
          .notify_when_done = false,
          .controlled = true,
          .params = {
            .rotation_data = {
              .target_rotation = arg
            }
          }
        };
        nav_add_action(&rotation_act);
      }
    
      if(mag > POS_THRESHOLD){
        action_t forward_act = {
          .type = NAV_ACTION_FORWARD,
          .target = NAV_TARGET_STATE,
          .notify_when_done = false,
          .controlled = true,
          .speed = NAV_FORWARD_SPEED_INITIAL,
          .params = {
            .forward_data = {
              .target_position = target_state.position
            }
          }
        };
        nav_add_action(&forward_act);
      }
    }else{
      msg_length = sprintf(buff, "END");
      ble_indicate_status(DCS_STATUS_LOG, buff, msg_length);
      nav_end();
    }
}


void nav_handle_milestone_reached(void){
  NRF_LOG_RAW_INFO("[nav] MILESTONE REACHED\n");
  
  motors_brake();

  m_current_milestone_idx++;
  
  m_action_queue_tail = m_action_queue_head;
  nav_set_current_action(get_idle_action());
  
  ble_indicate_milestone_reached(&current_state);
  //ble_indicate_next_milestone(&target_state);

  nav_process_next_milestone();
}

#define sign(x) (x >= 0 ? 1 : -1)

/* Checks if a ray coming from the current position
*  crosses the circle centered at target position
*  with radius POS_THRESHOLD. If it doesn't then, 
*  we need to correct motion, because the robot is 
*  projected to miss the milestone.
*/
bool nav_angle_satisfactory(float angle){
  if(angle >= MOTION_CORRECTION_ANGLE_THRESHOLD ) return false;
  if(angle <= m_rotation_thresh ) return true;
  
  // u = target - current
  point u = point_sub(target_state.position, current_state.position);
  
  // v = current unit direction vector
  point v = {.x = cos(current_state.rotation), .y = sin(current_state.rotation)};

  // Project u onto the ray
  // p = v * (u.v) / ||v|| ^ 2
  point p = point_scale(v, point_dot(u, v));

  // center to ray distance = ||u - p||
  p = point_sub(u, p);
  float distance = point_mag(&p);

  // Crosses circle if distance <= radius
  return distance <= POS_THRESHOLD + 1;
}

void nav_stop(void){
  motors_brake();
  nav_set_current_action(get_idle_action());
}


// Determines if robot has crossed plane perpendicular to adjacent milestones
// This avoids cases where robot overshoots or is close enough
//     |
//     |
// R - M1 <----- M2
//     |
//     |

bool nav_crossed_milestone_plane(int m_current_milestone_idx){
  point a = milestones[m_current_milestone_idx];
  point b = milestones[m_current_milestone_idx-1];
  return point_dot(point_sub(b, a), point_sub(current_state.position, b)) < 0; 
}

point proj(point a, point b) {
    float k = point_dot(a, b) / point_dot(b, b);
    return point_scale(b, k);// {x: k * b.x, y: k * b.y};
}

float hypot2(point a, point b){
  return point_dot(point_sub(a, b), point_sub(a, b));
}

bool nav_crossed_milestone_region(int m_current_milestone_idx){
    /**
   * Returns the distance from line segment AB to point C
   */
    // Compute vectors AC and AB
    point A = last_state_estimate.position;
    point B = current_state.position;
    point C = milestones[m_current_milestone_idx];
    
    point AC = point_sub(C, A);
    point AB = point_sub(B, A);

    // Get point D by taking the projection of AC onto AB then adding the offset of A
    point D = point_add(proj(AC, AB), A);

    point AD = point_sub(D, A);
    // D might not be on AB so calculate k of D down AB (aka solve AD = k * AB)
    // We can use either component, but choose larger value to reduce the chance of dividing by zero
    float k = abs(AB.x) > abs(AB.y) ? AD.x / AB.x : AD.y / AB.y;

    // Check if D is off either end of the line segment
    float distance;
    if (k <= 0.0) {
        distance = sqrt(hypot2(C, A));
    } else if (k >= 1.0) {
        distance = sqrt(hypot2(C, B));
    }else{
      distance = sqrt(hypot2(C, D));
    }

    return distance <= POS_THRESHOLD;
}

bool nav_milestone_reached(float distance){
  if(m_current_milestone_idx == num_milestones - 1){
    return distance <= POS_THRESHOLD;
  }else if(m_current_milestone_idx == 0){
    return distance <= POS_THRESHOLD; // First milestone cannot use previous state because it may be wrong
  }else{
    // Otherwise, check if segment joining current and previous positions has crossed the milestone
    return (distance <= POS_THRESHOLD) || (nav_crossed_milestone_region(m_current_milestone_idx)); 
  }
}

void nav_set_rotation_threshold(float thresh){
  m_rotation_thresh = thresh;
}

void nav_reset_rotation_threshold(void){
  m_rotation_thresh = ROTATION_THRESHOLD_DEFAULT;
}

void nav_update(void){
  if(!m_nav_enabled) return;
  
  //NRF_LOG_RAW_INFO("[nav] UPDATE\n");
  int64_t nav_update_current_time = systemTimeGetUs();
  float dt = (nav_update_current_time  - nav_update_previous_time) * 1e-6;
  nav_update_previous_time = nav_update_current_time ;
  
  // Check if action has reach timer limit
  if(current_action.type != NAV_ACTION_IDLE &&
     current_action.target == NAV_TARGET_TIME && 
     (nav_update_current_time - current_action.start_time) >= current_action.duration_us){    
    // If no other actions left in the queue, notify host of motion end
    bool notify = current_action.notify_when_done;
    bool has_next = nav_process_next_action();
    if(!has_next && notify){
      nav_end();
    }
  }
  
  switch(current_action.type){
    case NAV_ACTION_FORWARD:{
      #if LED_MARKER_ENABLED
        #ifdef USE_LED_FIXED_COLOR
        LED_FIXED_COLOR;
        #else
        led_rgb(0,1,1);
        #endif
      #else
        led_rgb(0,0,0);
      #endif
      
      if(current_action.controlled){

        point dir = point_sub(target_state.position, current_state.position);
        float distance = point_mag(&dir);
        #if DISTANCE_DEBUG
          NRF_LOG_RAW_INFO("[nav] Distance left: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(point_mag(&dir)));
        #endif
        
        if(current_action.target == NAV_TARGET_STATE && nav_milestone_reached(distance)){
          nav_handle_milestone_reached();
          bool has_next = nav_process_next_action();
          if(!has_next && current_action.notify_when_done){
            nav_end();
          }
        
        }else{
          float arg = point_arg(&dir) * 180 / M_PI;
          float angle_distance = fmod(arg + 360 - current_state.rotation, 360);
          if(angle_distance > 180) angle_distance = 360 - angle_distance;
        
          // If angle between robot and milestone is too large, stop and reset
          if(current_action.target == NAV_TARGET_STATE &&
             (nav_update_current_time - current_action.start_time) >= 1e6 &&
             !nav_angle_satisfactory(angle_distance)){
             motors_brake();
            //nav_process_next_milestone();
            //nav_process_next_action();
            //NRF_LOG_RAW_INFO("[nav] ANGLE DISTANCE: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(angle_distance));
            nav_set_current_action(get_idle_action());
            ble_indicate_motion_correction(&current_state);
            //ble_indicate_transition(NAV_ACTION_FORWARD, NAV_ACTION_IDLE);
            //eventQueuePush(EVENT_MOTION_CORRECTION);
          }else{
            #if GYRO_CONTROL_ENABLE
              current_yaw_rate = current_state.angular_velocity;
              float update_amount = pid_put_next(&stability_controller, current_yaw_rate, dt) * 0.0001; // Rate to update gain              
              
              if(nav_collision_detection_enabled &&
                 (nav_update_current_time - current_action.start_time) >= 1e6 && 
                 (beta < 0.35 || beta > 0.65)){
                 eventQueuePush(EVENT_COLLISION);
              }

              beta += update_amount;
              if(beta < 0.25) beta = 0.25;
              if(beta > 0.75) beta = 0.75;
           #if YAW_CONTROL_DEBUG
              NRF_LOG_RAW_INFO("[nav] dt: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(dt));
              NRF_LOG_RAW_INFO("[nav] Beta updated! " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(beta));
              NRF_LOG_RAW_INFO("[nav] Update amount (x1 scaled) " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(update_amount));
            #endif

            if(current_action.target == NAV_TARGET_STATE){
              current_state.mtr_val = pid_put_next(&forward_controller, distance, dt) * 22; // Rate to update gain
      
              if(fabs(current_state.mtr_val) >= 180){
                current_state.mtr_val = 180;
              }if(fabs(current_state.mtr_val) <= 140){
                current_state.mtr_val = 140;
              }
            }
          
            motors_adjust(current_state.mtr_val, beta);
            #endif
          }
        }
      }
    }
    break;
    
    case NAV_ACTION_ROTATE:{
      // CS -> TS 
      #if LED_MARKER_ENABLED
        #ifdef USE_LED_FIXED_COLOR
        LED_FIXED_COLOR;
        #else
        led_rgb(1,1,0);
        #endif
      #else
        led_rgb(0,0,0);
      #endif
      
      if(current_action.target == NAV_TARGET_STATE){
        float a = fmod(target_state.rotation + (360 - current_state.rotation), 360);
        float angle_error = a < 180 ? a : a - 360;

        float controlled_speed = pid_put_next(&rotation_controller, angle_error, dt) * 1.7; // Rate to update gain
    
        if(fabs(controlled_speed) >= 220){
          controlled_speed = sign(controlled_speed) * 220;
        }if(fabs(controlled_speed) <= 150){
          controlled_speed = sign(controlled_speed) * 150;
        }

        controlled_speed = controlled_speed + sign(controlled_speed) * (nav_update_current_time - current_action.start_time) * 5e-5; // Increase speed by 50 every second (to prevent stalls)

        #if NAV_ROTATE_DEBUG
        NRF_LOG_RAW_INFO("[nav] Speed: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(controlled_speed));
        NRF_LOG_RAW_INFO("[nav] Current rotation: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(current_state.rotation));
        NRF_LOG_RAW_INFO("[nav] Target rotation: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(target_state.rotation));
        NRF_LOG_RAW_INFO("[nav] Angle error: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(angle_error));
        #endif
        // If angle error is within threshold
        if(fabs(angle_error) < m_rotation_thresh){
          motors_hard_brake();
          NRF_LOG_RAW_INFO("[nav] ROTATION COMPLETE. MOVING ON TO NEXT ACTION\n");
          bool notify = current_action.notify_when_done;
          bool has_next = nav_process_next_action();
          if(!has_next && notify){
            nav_end();
          }
        }else if(controlled_speed < 0){
          // It's easier to go from CS to TS (positive sense)
          motors_rotate_ccw_direct(-controlled_speed);
        }else{
          // It's easier to go from TS to CS (negative sense)
          motors_rotate_cw_direct(controlled_speed);
        }
      }else{
        if(current_action.speed > 0){
          motors_rotate_ccw_direct(current_action.speed);
        }else{
          motors_rotate_ccw_direct(-current_action.speed);
        }
      }
    }
    break;
    
    case NAV_ACTION_BACKWARD:
    // TODO: Maybe do rotation correction here?
    //current_yaw_rate = current_state.angular_velocity;
    //float update_amount = pid_put_next(&stability_controller, current_yaw_rate, dt) * 0.0001; // Rate to update gain
    //beta += update_amount;
    //if(beta < 0) beta = 0;
    //if(beta > 1) beta = 1;
    //motors_adjust(current_state.mtr_val, beta);
    break;

    case NAV_ACTION_RESPONSIVE:
    break;

    case NAV_ACTION_ROTATE_PRECISE:{
      float a = fmod(target_state.rotation + (360 - current_state.rotation), 360);
      float angle_error = a < 180 ? a : a - 360;
      float controlled_speed = pid_put_next(&rotation_controller, angle_error, dt); // Rate to update gain
      controlled_speed = sign(controlled_speed) * (120 + (nav_update_current_time - current_action.start_time) * 1e-5); // Increase speed by 10 every second (to prevent stalls)
      
      NRF_LOG_RAW_INFO("[nav] Speed: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(controlled_speed));
      if(fabs(angle_error) <= 2){
        motors_hard_brake();
        bool notify = current_action.notify_when_done;
        bool has_next = nav_process_next_action();
        if(!has_next && notify){
          nav_end();
        }
      }else if(controlled_speed < 0){
        // It's easier to go from CS to TS (positive sense)
        motors_rotate_ccw_direct(-controlled_speed);
      }else{
        // It's easier to go from TS to CS (negative sense)
        motors_rotate_cw_direct(controlled_speed);
      }
    }
    break;

    default:
      #if LED_MARKER_ENABLED
        #ifdef USE_LED_FIXED_COLOR
        LED_FIXED_COLOR;
        #else
        led_rgb(0,0,1);
        #endif
      #else
        led_rgb(0,0,0);
      #endif
      nav_process_next_action();
    break;
  }
}

void nav_reset_beta(void){
  beta = 0.5;
}

void nav_acc_update(void){
  if(!m_nav_enabled) return;

  NRF_LOG_RAW_INFO("[nav] ACCEL UPDATE\n");
  
  if(current_action.type != NAV_ACTION_FORWARD){
    nav_update();
    return;
  }

  if(acc_previous_time == 0){
    acc_previous_time = systemTimeGetUs();
    return;
  }

  // Time update
  int64_t acc_current_time = systemTimeGetUs();
  float dt = (acc_current_time - acc_previous_time) * 1e-6;
  acc_previous_time = acc_current_time;
  
  float data[3];
  accel_get_xyz(data);

  point acc;

  point dir = {.x = cos(current_state.rotation * M_PI / 180), .y = sin(current_state.rotation * M_PI / 180)};
  
  // Robot moves in +x direction. Hence, only the x-compenent of the acceleration should
  // contribute to robot position motion.
  
  #if USE_PITCH_COMPENSATION
    float theta = current_state.pitch * M_PI / 180;
    float azr = 1000;
    float x1 = data[0] + azr * sin(theta);
    float x2 = azr * cos(theta) - data[2];
    float ax = sqrt(x1 * x1 + x2 * x2);
    //float az = data[2] * ( 1 - cos(current_state.pitch * M_PI / 180) ); // Compensate pitch on z
    float scale = sign(data[0]) * ax; //sqrt(az * az + ax * ax);
    acc = point_scale(dir, scale);
  #else
    acc = point_scale(dir, data[0]);
  #endif

  acc = point_scale(acc, 0.98 * dt); // v = at + v0

  //NRF_LOG_RAW_INFO("[nav] ax: ("NRF_LOG_FLOAT_MARKER", az=" NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(data[0]), NRF_LOG_FLOAT(data[2]))

  current_state.velocity = point_add(current_state.velocity, acc);

  // x = 0.5 * a * t^2 + v_0t + x_0
  current_state.position = point_add(current_state.position, point_scale(acc, 0.5 * dt));
  current_state.position = point_add(current_state.position, point_scale(current_state.velocity, dt));
  
  if(point_dot(dir, current_state.velocity) > 0){
    current_state.velocity = point_project(current_state.velocity, dir);
  }else{
    current_state.velocity.x = current_state.velocity.y = 0;
  }
  
  //NRF_LOG_RAW_INFO("%08d \n", systemTimeGetUs());
  //NRF_LOG_RAW_INFO("[nav] ax: ("NRF_LOG_FLOAT_MARKER", az=" NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(data[0]), NRF_LOG_FLOAT(data[2]))

 //NRF_LOG_RAW_INFO("[nav] dt: "NRF_LOG_FLOAT_MARKER"\n",
 //   NRF_LOG_FLOAT(dt * 1e6))
 //NRF_LOG_RAW_INFO("[nav] Pos update cm: ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n",
 //   NRF_LOG_FLOAT(pos_update.x),
 //   NRF_LOG_FLOAT(pos_update.y))

 //NRF_LOG_RAW_INFO("[nav] Current accel (cm/s^2): ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(acc.x), NRF_LOG_FLOAT(acc.y))
 // NRF_LOG_RAW_INFO("[nav] Current velocity (cm/s): ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(current_state.velocity.x), NRF_LOG_FLOAT(current_state.velocity.y))
 // NRF_LOG_RAW_INFO("[nav] Current pos (cm): ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(current_state.position.x), NRF_LOG_FLOAT(current_state.position.y))

#if ESTIMATED_POSITION_DEBUG
  
  NRF_LOG_RAW_INFO("[nav] Current accel (cm/s^2): ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(acc.x), NRF_LOG_FLOAT(acc.y))
  NRF_LOG_RAW_INFO("[nav] Current velocity (cm/s): ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(current_state.velocity.x), NRF_LOG_FLOAT(current_state.velocity.y))
  NRF_LOG_RAW_INFO("[nav] Current pos (cm): ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n", NRF_LOG_FLOAT(current_state.position.x), NRF_LOG_FLOAT(current_state.position.y))
  
#endif

  //NRF_LOG_RAW_INFO("[nav] Current dv: ("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") \n",
  //    NRF_LOG_FLOAT(acc.x),
  //    NRF_LOG_FLOAT(acc.y))

  nav_update();
}

void nav_gyro_update(void){
  int64_t gyro_current_time = systemTimeGetUs();

  // Only update within 400ms after switching to idle to prevent drift due to action switching
  if(current_action.type == NAV_ACTION_IDLE && (gyro_current_time - current_action.start_time) >= 4e5 ){
    nav_update();
    return;
  }

  //if(current_action.type == NAV_ACTION_IDLE ){
    //NRF_LOG_RAW_INFO("[nav] Still updating!\n");
  //}

  if(gyro_previous_time == 0){
    gyro_previous_time = gyro_current_time;
    return;
  }
  // Time update
  float dt = (gyro_current_time - gyro_previous_time) * 1e-6;
  gyro_previous_time = gyro_current_time;

  float yaw_rate = gyro_get_yaw_rate() / 1000.0; // mdps to dps
  current_state.unwrapped_rotation += (yaw_rate + current_state.angular_velocity) * 0.5 * dt;
  current_state.rotation = fmod(360 + fmod(current_state.unwrapped_rotation, 360), 360);
  
  float pitch_rate = gyro_get_pitch_rate() / 1000.0; // mdps to dps
  current_state.pitch += (pitch_rate + current_state.pitch_velocity) * 0.5 * dt;

  current_state.angular_velocity = yaw_rate;
  current_state.pitch_velocity = pitch_rate;

#if ESTIMATED_ROTATION_DEBUG
  NRF_LOG_RAW_INFO("[nav] Current rotation (deg): "NRF_LOG_FLOAT_MARKER"\n", NRF_LOG_FLOAT(current_state.unwrapped_rotation));
  NRF_LOG_RAW_INFO("[nav] Current pitch (deg): "NRF_LOG_FLOAT_MARKER"\n", NRF_LOG_FLOAT(current_state.pitch));
#endif

  nav_update();
}

float nav_wrap_angle(float x){
  return fmod(360 + fmod(360 + x, 360), 360);
}

// Updates current state with acoustic position estimate
void nav_position_update(nav_state_t *estimated_state, nav_state_t *current_milestone, nav_state_t *next_milestone, uint8_t flags){  
  if(flags & (1 << NAV_FLAG_FORCE_UPDATE)){
    current_state.position = estimated_state->position;

    current_state.unwrapped_rotation = nav_wrap_angle(estimated_state->unwrapped_rotation);
    current_state.rotation = current_state.unwrapped_rotation;

    last_state_estimate = current_state;
  }else{
    if(!m_nav_enabled) return;
    #if POSITION_UPDATES_DEBUG
    NRF_LOG_RAW_INFO("[nav] Estimated State: p=("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") | r=",
                    NRF_LOG_FLOAT(estimated_state->position.x),
                    NRF_LOG_FLOAT(estimated_state->position.y));
    NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER"\n", NRF_LOG_FLOAT(estimated_state->rotation));
    NRF_LOG_RAW_INFO("[nav] Estimated Velocity: v=("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER"\n",
                    NRF_LOG_FLOAT(estimated_state->velocity.x),
                    NRF_LOG_FLOAT(estimated_state->velocity.y));

    NRF_LOG_RAW_INFO("[nav] Current Milestone: p=("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") | r=",
                    NRF_LOG_FLOAT(current_milestone->position.x),
                    NRF_LOG_FLOAT(current_milestone->position.y));
    NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER"\n", NRF_LOG_FLOAT(current_milestone->rotation));

    //NRF_LOG_RAW_INFO("[nav] Next Milestone: p=("NRF_LOG_FLOAT_MARKER", " NRF_LOG_FLOAT_MARKER") | r=",
    //                NRF_LOG_FLOAT(next_milestone->position.x),
    //                NRF_LOG_FLOAT(next_milestone->position.y));
    //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER"\n", NRF_LOG_FLOAT(next_milestone->rotation));
    #endif

    if(current_action.type != NAV_ACTION_ROTATE){
      point estimated_velocity = estimated_state->velocity;
  
      float estimated_rotation = nav_wrap_angle(estimated_state->unwrapped_rotation);

      point angular_mean = {
        .x = cos(current_state.unwrapped_rotation * M_PI / 180),
        .y = sin(current_state.unwrapped_rotation * M_PI / 180),
      };

      point est_angular_cpx = {
        .x = cos(estimated_rotation * M_PI / 180),
        .y = sin(estimated_rotation * M_PI / 180),
      };

      if(current_action.type == NAV_ACTION_FORWARD){
        // Weighted sum of local velocity estimation & acoustic velocity estimation
        float wv = 0.8;
        current_state.velocity = point_add(point_scale(current_state.velocity, 1-wv), point_scale(estimated_velocity, wv));
        if(point_dot(current_state.velocity, angular_mean) > 0){
          current_state.velocity = point_project(current_state.velocity, angular_mean);
        }else{
          current_state.velocity.x = current_state.velocity.y = 0;
        }
      }
      
      // Weighted sum of local rotation estimation & acoustic rotation estimation
      if(flags & (1 << NAV_FLAG_UPDATE_ANGLE)){
        float wr = 0.2;
        //float wr = 1;
        //NRF_LOG_INFO("UPDATED ANGLE!!**\n");
        angular_mean = point_add(point_scale(angular_mean, wr), point_scale(est_angular_cpx, 1-wr));
        current_state.unwrapped_rotation = nav_wrap_angle(atan2(angular_mean.y, angular_mean.x) * 180 / M_PI);
        current_state.rotation = current_state.unwrapped_rotation;
      }
    }
  
    if(current_action.type != NAV_ACTION_ROTATE){
      // Weighted sum of local position estimation & acoustic position estimation
      float wp = 1;
      estimated_state->position = point_scale(estimated_state->position, wp);
      current_state.position = point_scale(current_state.position, 1-wp);
      current_state.position = point_add(estimated_state->position, current_state.position);
  
      last_state_estimate.position.x = current_state.position.x;
      last_state_estimate.position.y = current_state.position.y;
  
      last_state_estimate.velocity.x = current_state.velocity.x;
      last_state_estimate.velocity.y = current_state.velocity.y;

      last_state_estimate.unwrapped_rotation = current_state.unwrapped_rotation;
    }
    if(current_action.type == NAV_ACTION_IDLE){
      nav_process_next_milestone();
    }

    point dir = point_sub(target_state.position, estimated_state->position);
    float distance = point_mag(&dir);

    //float pos_thresh = POS_THRESHOLD;
    //if(m_current_milestone_idx == num_milestones - 1){
    //  pos_thresh = pos_thresh / 2;
    //}

    if(nav_milestone_reached(distance)){
      float wp = 1;
      estimated_state->position = point_scale(estimated_state->position, wp);
      current_state.position = point_scale(current_state.position, 1-wp);
      current_state.position = point_add(estimated_state->position, current_state.position);
      current_state.rotation = fmod(360 + fmod(360 + current_state.unwrapped_rotation, 360), 360);

      last_state_estimate.position.x = current_state.position.x;
      last_state_estimate.position.y = current_state.position.y;

      last_state_estimate.velocity.x = current_state.velocity.x;
      last_state_estimate.velocity.y = current_state.velocity.y;

      last_state_estimate.unwrapped_rotation = current_state.unwrapped_rotation;
      last_state_estimate.pitch = current_state.pitch;
      nav_handle_milestone_reached();
      nav_process_next_action();
    }
    m_debug_flag = true;
    nav_update();
    m_debug_flag = false;
  }
  
}

void nav_forward(uint64_t duration_us, int16_t speed, bool controlled){
  uint8_t args[sizeof(uint64_t) + sizeof(int16_t) + sizeof(bool)];
  uint8_t *cursor = args;
  
  ((uint64_t *) cursor)[0] = duration_us; cursor += sizeof(uint64_t);
  ((int16_t *) cursor)[0] = speed; cursor += sizeof(int16_t);
  ((bool *) cursor)[0] = controlled; cursor += sizeof(bool);

  nav_motion_update(MOTION_FORWARD, args, cursor - args);
}

void nav_backward(uint64_t duration_us, int16_t speed, bool controlled){
  uint8_t args[sizeof(uint64_t) + sizeof(int16_t) + sizeof(bool)];
  uint8_t *cursor = args;
  
  ((uint64_t *) cursor)[0] = duration_us; cursor += sizeof(uint64_t);
  ((int16_t *) cursor)[0] = speed; cursor += sizeof(int16_t);
  ((bool *) cursor)[0] = controlled; cursor += sizeof(bool);

  nav_motion_update(MOTION_BACKWARD, args, cursor - args);
}

void nav_rotate_angle(float angle){
  uint8_t args[sizeof(float)];
  uint8_t *cursor = args;
  
  ((float *) cursor)[0] = angle; cursor += sizeof(float);

  nav_motion_update(MOTION_ROTATE_ANGLE, args, cursor - args);
}

void nav_rotate_angle_precise(float angle){
  uint8_t args[sizeof(float)];
  uint8_t *cursor = args;
  
  ((float *) cursor)[0] = angle; cursor += sizeof(float);

  nav_motion_update(MOTION_ROTATE_ANGLE_PRECISE, args, cursor - args);
}

void nav_rotate_time(uint64_t duration_us, int16_t speed){
  uint8_t args[sizeof(uint64_t) + sizeof(int16_t)];
  uint8_t *cursor = args;
  
  ((uint64_t *) cursor)[0] = duration_us; cursor += sizeof(uint64_t);
  ((int16_t *) cursor)[0] = speed; cursor += sizeof(int16_t);

  nav_motion_update(MOTION_ROTATE_TIME, args, cursor - args);
}

void nav_motion_update(motion_code_t code, const uint8_t * const args, uint8_t length){
  //nav_stop();
  
  if(code != MOTION_BRAKE) nav_begin();
  
  switch(code){
    case MOTION_FORWARD:{
    //motors_forward(speed);
    //nav_controlled_forward(speed);
    uint64_t duration = ((uint64_t *) args)[0];
    int16_t speed = (length > sizeof(uint64_t) ? ((int16_t *) (args + sizeof(uint64_t)))[0] : NAV_FORWARD_SPEED_INITIAL);
    bool controlled = (length > sizeof(uint64_t) + sizeof(int16_t) ? args[sizeof(uint64_t) + sizeof(int16_t)] : 1);
    action_t action = {
      .type = NAV_ACTION_FORWARD,
      .target = NAV_TARGET_TIME,
      .notify_when_done = true,
      .controlled = controlled,
      .duration_us = duration,
      .speed = speed,
    };
    nav_add_action(&action);
    }
    break;

    case MOTION_BACKWARD:{
    uint64_t duration = ((uint64_t *) args)[0];
    int16_t speed = (length > sizeof(uint64_t) ? ((int16_t *) (args + sizeof(uint64_t)))[0] : NAV_FORWARD_SPEED_INITIAL);
    bool controlled = args[sizeof(uint64_t) + sizeof(int16_t)];
      action_t action = {
        .type = NAV_ACTION_BACKWARD,
        .target = NAV_TARGET_TIME,
        .notify_when_done = true,
        .controlled = controlled,
        .duration_us = duration,
        .speed = speed,
      };
      
      nav_add_action(&action);
    }
    break;
    
    case MOTION_ROTATE_ANGLE:{
      float angle = ((float *) args)[0];
      action_t action = {
        .type = NAV_ACTION_ROTATE,
        .target = NAV_TARGET_STATE,
        .notify_when_done = true,
        .params = {
            .rotation_data = {
              .target_rotation = angle
            }
          }
        };
      nav_add_action(&action);
    }
    break;
    
    case MOTION_ROTATE_TIME:{
      uint64_t duration = ((uint64_t *) args)[0];
      int16_t speed = (length > sizeof(uint64_t) ? ((int16_t *) (args + sizeof(uint64_t)))[0] : NAV_FORWARD_SPEED_INITIAL);
      action_t action = {
        .type = NAV_ACTION_ROTATE,
        .target = NAV_TARGET_TIME,
        .notify_when_done = true,
        .speed = speed,
        .duration_us = duration,
      };
      nav_add_action(&action);
    }
    break;
    case MOTION_ROTATE_ANGLE_PRECISE:{
      float angle = ((float *) args)[0];
      action_t action = {
        .type = NAV_ACTION_ROTATE_PRECISE,
        .target = NAV_TARGET_STATE,
        .notify_when_done = true,
        .params = {
            .rotation_data = {
              .target_rotation = angle
            }
          }
        };
      nav_add_action(&action);
    }
    break;
    case MOTION_BRAKE:
      motors_brake();
      nav_end();
      nav_clear_action_queue();
    break;
    case MOTION_FORWARD_PULSE:
      motors_pulse_forward(100, 200, 100);
    break;
    case MOTION_BACKWARD_PULSE:
      motors_pulse_backward(100, 200, 100);
    break;
  }
  if(current_action.type == NAV_ACTION_IDLE){
    nav_process_next_action();
  }
}

nav_state_t nav_get_current_state(void){
  return current_state;
}

nav_state_t nav_get_previous_state_estimate(void){
  return last_state_estimate;
}

action_t nav_get_current_action(void){
  return current_action;
}

void nav_clear_milestones(void){
  m_current_milestone_idx = 0;
  num_milestones = 0;
}

void nav_reset(void){
  m_nav_enabled = false;
  nav_clear_milestones();
  nav_set_current_action(get_idle_action());
}

void nav_begin(void){
  m_nav_enabled = true;
  //nav_process_next_milestone();
}

void nav_end(void){
  nav_reset();
  float data[5];
  
  NRF_LOG_RAW_INFO("NAV END\n");

  nav_collision_detection_enabled = false;

  // Send nav end
  uint8_t length = ble_dump_nav_state(data, &current_state);
  ble_indicate_status(DCS_STATUS_NAV_END, (uint8_t*) data, length);
}

void nav_update_pitch(void){
  current_state.pitch = accel_get_pitch();
  current_state.pitch_velocity = 0;

  last_state_estimate.pitch = current_state.pitch;
  last_state_estimate.pitch_velocity = 0;
}

void nav_init(void){
  current_state.position.x = INITIAL_POSITION_X;
  current_state.position.y = INITIAL_POSITION_Y;
  
  current_state.rotation = INITIAL_ROTATION;
  current_state.unwrapped_rotation = INITIAL_ROTATION;
  current_state.angular_velocity = 0;
  current_state.pitch = accel_get_pitch();
  current_state.pitch_velocity = 0;
  current_state.mtr_val = 0;
  
  current_state.velocity.x = 0;
  current_state.velocity.y = 0;

  last_state_estimate.position.x = current_state.position.x;
  last_state_estimate.position.y = current_state.position.y;
  
  last_state_estimate.velocity.x = current_state.velocity.x;
  last_state_estimate.velocity.y = current_state.velocity.y;

  last_state_estimate.pitch = current_state.pitch;
  last_state_estimate.pitch_velocity = 0;

  last_state_estimate.unwrapped_rotation = current_state.unwrapped_rotation;

  target_state = current_state;

  nav_reset();
}

void nav_send_all(uint16_t sample_offset, recording_t *rec){
  static uint8_t data[256];
  
  ble_dump_xcorr_offset(data, sample_offset);

  float *values = ((float*) data) + 1;
  ble_dump_nav_state(values, &current_state); values += 5;
  
  ble_dump_nav_state(values, &last_state_estimate); values += 5;

  ble_dump_nav_state(values, &target_state); values += 5;

  ((uint64_t*) values)[0] = rec->start_time;
  ((uint64_t*) values)[1] = rec->end_time;
  values += 4;

  ((uint8_t*) values)[0] = current_action.type;
  ((uint8_t*) values)[1] = 0xFF;
  ((uint8_t*) values)[2] = 0xFF;
  ((uint8_t*) values)[3] = 0xFF;
  values++;
  
  if(xcorr_stream_use_notifications){
    NRF_LOG_RAW_INFO("Using notifications!!\n");
    ble_notify_status(DCS_STATUS_UPDATE_XCORR, data, ((uint8_t *) values) - data);
  }else{
    NRF_LOG_RAW_INFO("Using indications!!\n");
    ble_indicate_status(DCS_STATUS_UPDATE_XCORR, data, ((uint8_t *) values) - data);
  }
}

void nav_recover(void){
  motors_hard_brake();
  nav_clear_action_queue();
  uint8_t args[sizeof(uint64_t) + sizeof(int16_t)];
  ((uint64_t *)args)[0] = 500e3;
  ((int16_t *)args)[4] = 150;
  nav_motion_update(MOTION_BACKWARD, args, sizeof(uint64_t) + sizeof(int16_t));
  ir_disable();
}

void nav_disable_collision_detection(void){
  nav_collision_detection_enabled = false;
}

void nav_enable_collision_detection(void){
  nav_collision_detection_enabled = true;
}