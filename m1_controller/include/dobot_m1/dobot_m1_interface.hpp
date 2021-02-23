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
void ConnectDobot(const std::string &port_name, uint32_t baudrate) throw(std::runtime_error);
bool TryConnectDobot(const std::string &port_name, uint32_t baudrate);

void SetCmdTimeout(const uint32_t timeout) throw(std::runtime_error);
bool TrySetCmdTimeout(const uint32_t timeout);

void SetQueuedCmdClear() throw(std::runtime_error);
bool TrySetQueuedCmdClear();

void CheckConnectionWithException(const std::string &called_from, const uint8_t status) throw(std::runtime_error);
bool CheckConnection(const uint8_t status);
void ConnectionStatus2String(const uint8_t status, std::string &str);

void CheckCommunication(const std::string &called_from, const uint8_t status) throw(std::runtime_error);
bool CheckCommunication(const uint8_t status);
void CommunicationStatus2String(const uint8_t status, std::string &str);

}  // namespace dobot_m1_interface