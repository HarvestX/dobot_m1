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