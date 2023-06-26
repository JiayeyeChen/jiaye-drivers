#ifndef TMOTOR_AK10_9_V2_H
#define TMOTOR_AK10_9_V2_H

#include "system.h"
#include "common.h"
#include "my_math.h"
#include <math.h>

#define AK10_9_SPECIAL_COMMAND_ENABLE_MOTOR  0U
#define AK10_9_SPECIAL_COMMAND_DISABLE_MOTOR 1U
#define AK10_9_SPECIAL_COMMAND_ZEROING_MOTOR 2U

#define SIZE_OF_MOVING_ACC_AVG_BUFFER        25U

enum AK10_9_MITMode_MotorEnablingStatus
{
  AK10_9_MITMODE_ENABLED,
  AK10_9_MITMODE_DISABLED
};

enum AK10_9_OnlineStatus
{
  AK10_9_Online,
  AK10_9_Offline
};

typedef struct
{
  CAN_HandleTypeDef*                        hcan;
  uint32_t                                  canID;
  int8_t                                    temperature;
  uint8_t                                   errorCode;
  enum AK10_9_OnlineStatus                  status;
  enum AK10_9_MITMode_MotorEnablingStatus   enablingStatus;
  
  float                 kt;
  float                 posOffsetDeg;
  float                 posOffsetRad;
  float                 posDirectionCorrection;
  union FloatUInt8      setPos, goalPos;
  union FloatUInt8      setVel, goalVel;
  union FloatUInt8      setIq, goalIq;
  union FloatUInt8      setKp, goalKp;
  union FloatUInt8      setKd, goalKd;
  union FloatUInt8      realCurrent;
  union FloatUInt8      realPosition;
  union FloatUInt8      realPositionOffseted;
  union FloatUInt8      realPositionOffsetedRad;
  union FloatUInt8      realVelocityPresent;
  union FloatUInt8      realVelocityPresentRad;
  union FloatUInt8      realVelocityPrevious[2];
  union FloatUInt8      realTorque;
  union FloatUInt8      setAcceleration;
  union FloatUInt8      setAcceleration_ByRealPosition;
  union FloatUInt8      realAccelerationRaw;
  union FloatUInt8      realAccelerationFiltered;
  union FloatUInt8      realAccelerationFilteredRad;
  uint8_t               ifCustomizedPositionSpeedControlFinished;
  uint8_t               ifMITModeParameterSmootherWorkFinished;
  //For acceleration estimation//
  //Moving average value method//
  float                 accAverage;
  float                 accAverageBuf[SIZE_OF_MOVING_ACC_AVG_BUFFER];
  uint8_t               accAvgPtr;
  ///////////////////////////////
  //Low pass filter method//
  float                 realAccelerationFilteredPrevious;
  float                 cutOffFrequency;
  float                 alpha;
  float                 timeDuration;
  //////////////////////////
  //Butterworth filter method//
  float                 a2Butter, a3Butter, b1Butter, b2Butter, b3Butter;
  float                 realAccelerationFilteredPreviousButter[2];
  float                 realAccelerationRawPreviousButter[2];
  /////////////////////////////
  //CAN BUS transmit
  uint8_t               txBuf[8];
  uint32_t              txMailbox;
  CAN_TxHeaderTypeDef   txHeader;
  //CAN BUS receive
  uint8_t               rxBuf[8];
  uint32_t              rxFifo;
  CAN_FilterTypeDef     rxFilter;
  uint32_t              lastReceivedTime;
}AK10_9HandleCubeMarsFW;

AK10_9HandleCubeMarsFW AK10_9_Create(CAN_HandleTypeDef* hcan, uint8_t can_id, float kt, float dir, \
                                        float low_pass_filter_cut_off_frequency, float low_pass_filter_time_duration, \
                                        float butterworth_filter_a2, float butterworth_filter_a3, \
                                        float butterworth_filter_b1, float butterworth_filter_b2, float butterworth_filter_b3);
void AK10_9_ServoMode_CurrentControl(AK10_9HandleCubeMarsFW* hmotor, float current);
void AK10_9_ServoMode_VelocityControl(AK10_9HandleCubeMarsFW* hmotor, float speed);
void AK10_9_ServoMode_PositionControl(AK10_9HandleCubeMarsFW* hmotor, float position);
void AK10_9_ServoMode_PositionSpeenControlCustomized(AK10_9HandleCubeMarsFW* hmotor, float position, float speed, float loop_duration);
void AK10_9_ServoMode_PositionControlWithOffset(AK10_9HandleCubeMarsFW* hmotor, float position);
void AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(AK10_9HandleCubeMarsFW* hmotor, float position, float speed, float loop_duration);
void AK10_9_ServoMode_PositionSpeedControl(AK10_9HandleCubeMarsFW* hmotor, float position, float speed, int16_t acceleration);
void AK10_9_ServoMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleCubeMarsFW* hmotor, uint8_t rxbuf[]);
void AK10_9_ServoMode_Zeroing(AK10_9HandleCubeMarsFW* hmotor);
void AK10_9_MITMode_EnableMotor(AK10_9HandleCubeMarsFW* hmotor);
void AK10_9_MITMode_DisableMotor(AK10_9HandleCubeMarsFW* hmotor);
void AK10_9_CubeMarsFW_MITMode_ZeroingControlParameters(AK10_9HandleCubeMarsFW* hmotor);
void AK10_9_MITMode_Zeroing(AK10_9HandleCubeMarsFW* hmotor);
void AK10_9_MITModeControl_Deg(AK10_9HandleCubeMarsFW* hmotor, float pos, float vel, float kp, float kd, float iq);
void AK10_9_MITModeControl_Rad(AK10_9HandleCubeMarsFW* hmotor, float pos, float vel, float kp, float kd, float iq);
void AK10_9_MITModeCurrentControl(AK10_9HandleCubeMarsFW* hmotor, float iq);
void AK10_9_MITMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleCubeMarsFW* hmotor, uint8_t rxbuf[]);
void AK10_9_CubeMarsFW_MotorStatusMonitor(AK10_9HandleCubeMarsFW* hmotor, uint32_t timeout_ms);
void AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(AK10_9HandleCubeMarsFW* hmotor, \
                                                        float pos_slope, float vel_slope, float iq_slope, \
                                                        float kp_slope, float kd_slope, float loop_duration_ms);
void AK10_9_CubaMarsFW_MITMode_ContinuousControl_Deg(AK10_9HandleCubeMarsFW* hmotor, float goal_pos, float goal_vel, \
                                                 float goal_kp, float goal_kd, float goal_iq);
void AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(AK10_9HandleCubeMarsFW* hmotor, float goal_pos, float goal_vel, \
                                                               float goal_kp, float goal_kd, float goal_iq);
uint16_t FloatToUint(float x, float x_min, float x_max, uint16_t bits);
float    UintToFloat(uint16_t x_int, float x_min, float x_max, uint16_t bits);



/*Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump*/
//void AK10_9_MITMode_PositionSpeedControlCustomized_Deg(AK10_9HandleCubeMarsFW* hmotor, float position, float speed, float kp, float kd, float loop_duration);
//void AK10_9_MITMode_PositionSpeedControlCustomizedWithOffset_Deg(AK10_9HandleCubeMarsFW* hmotor, float position, float speed, float kp, float kd, float loop_duration);
//void AK10_9_DMFW_MITMode_PositionSpeedControlCustomized_Rad(AK10_9HandleDMFW* hmotor, float position, float speed, float kp, float kd, float loop_duration);
//typedef struct
//{
//  uint8_t ifCustomizedPositionSpeedControlStarted;
//}AK10_9HandleCubeMarsFW;
/*Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump*/


#endif
