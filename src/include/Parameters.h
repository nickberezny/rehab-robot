/**
 * @file Parameters.h
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Parameter macros
 *
 */

#define NUMBER_OF_THREADS 3
#define NSEC_IN_SEC 1000000000
#define BUFFER_SIZE 10
#define STEP_SIZE_MS 10
#define DAQ_NUM_OF_CH 5

#define WAIT_STATE 0
#define HOME_STATE 1
#define CALIBRATE_STATE 2
#define SET_STATE 3
#define READY_STATE 4
#define RUN_STATE 5
#define STOP_STATE 6
#define SHUTDOWN_STATE 7

#define ADDR "192.168.0.93"

//TODO
#define MOTOR_ZERO 0
#define ENC_TO_MM 0
#define MOTOR_SLOW_BWD 0
#define MOTOR_SLOW_FWD 0