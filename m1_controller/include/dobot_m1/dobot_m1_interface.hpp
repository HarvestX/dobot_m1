#pragma once

#include <stdexcept>
#include <string>

#include "dobot_m1_alarm.hpp"
namespace dobot_api
{
#include "DobotDll.h"
}

namespace dobot_m1_interface
{
enum {
  REAR = 1,
  FRONT = 2,
  END = 3,
};


const float ANGLE_EPSILON = 0.5;

const float DOBOT_REAR_JOINT_SOFTWARE_LIMIT = 85.0;
const float DOBOT_REAR_JOINT_HARDWARE_LIMIT = 90.0;

const float DOBOT_FORE_JOINT_SOFTWARE_LIMIT = 135.0;
const float DOBOT_FORE_JOINT_HARDWARE_LIMIT = 140.0;


const float DOBOT_END_JOINT_SOFTWARE_LIMIT = 360.0;
// END JOINT HARDWARE LIMIT is None

void ConnectDobot(const std::string &port_name, uint32_t baudrate);
bool TryConnectDobot(const std::string &port_name, uint32_t baudrate);

void SetCmdTimeout(const uint32_t timeout);
bool TrySetCmdTimeout(const uint32_t timeout);

void SetQueuedCmdClear();
bool TrySetQueuedCmdClear();

void ClearAllAlarmsState();
bool TryClearAllAlarmsState();

void SetQueuedCmdStartExec();
bool TrySetQueuedCmdStartExec();

void SetArmOrientationRight();
bool TrySetArmOrientationRight();

void SetArmOrientationLeft();
bool TrySetArmOrientationLeft();

void SetPtpCmd(uint8_t mode, float x, float y, float z, float r);
bool TrySetPtpCmd(uint8_t mode, float x, float y, float z, float r);

void SetCpCmd(uint8_t mode, float x, float y, float z);
bool TrySetCpCmd(uint8_t mode, float x, float y , float z);

void SetJogParams(float vel, float acc);
bool TrySetJogParams(float vel, float acc);
bool TryIdleJog();

void WaitQueuedCmd(const uint64_t last_index);
bool TryWaitQueuedCmd(const uint64_t last_index);

void GetAlarmCode(uint32_t &code);
bool TryGetAlarmCode(uint32_t &code);
bool CheckAlarmCode(const uint32_t &code);
void AlarmCode2String(const uint32_t &code, std::string &str);

void CheckConnectionWithException(const std::string &called_from, const uint8_t status);
bool CheckConnection(const uint8_t status);
void ConnectionStatus2String(const uint8_t status, std::string &str);

void CheckCommunicationWithException(const std::string &called_from, const uint8_t status);
bool CheckCommunication(const uint8_t status);
void CommunicationStatus2String(const uint8_t status, std::string &str);

}  // namespace dobot_m1_interface