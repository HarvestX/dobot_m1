#include "dobot_m1_interface.hpp"

namespace dobot_m1_interface
{
void ConnectDobot(const std::string &port_name, uint32_t baudrate)
{
  const uint8_t status = dobot_api::ConnectDobot(port_name.c_str(), baudrate, nullptr, nullptr);
  CheckConnectionWithException("ConnectDobot", status);
  return;
}

bool TryConnectDobot(const std::string &port_name, uint32_t baudrate)
{
  const uint8_t status = dobot_api::ConnectDobot(port_name.c_str(), baudrate, nullptr, nullptr);
  return CheckConnection(status);
}

void SetCmdTimeout(const uint32_t timeout)
{
  const uint8_t status = dobot_api::SetCmdTimeout(timeout);
  CheckCommunicationWithException("SetCmdTimeout", status);
  return;
}

bool TrySetCmdTimeout(const uint32_t timeout)
{
  const uint8_t status = dobot_api::SetCmdTimeout(timeout);
  return CheckCommunication(status);
}

void SetQueuedCmdClear()
{
  const uint8_t status = dobot_api::SetQueuedCmdClear();
  CheckCommunicationWithException("SetQueuedCmdClear", status);
  return;
}

bool TrySetQueuedCmdClear()
{
  const uint8_t status = dobot_api::SetQueuedCmdClear();
  return CheckCommunication(status);
}

void ClearAllAlarmsState()
{
  const uint8_t status = dobot_api::ClearAllAlarmsState();
  CheckCommunicationWithException("ClearAllAlarmsState", status);
  return;
}

bool TryClearAllAlarmsState()
{
  const uint8_t status = dobot_api::ClearAllAlarmsState();
  return CheckCommunication(status);
}

void SetQueuedCmdStartExec()
{
  const uint8_t status = dobot_api::SetQueuedCmdStartExec();
  CheckCommunicationWithException("SetQUeuedCmdStartExec", status);
  return;
}

bool TrySetQueuedCmdStartExec()
{
  const uint8_t status = dobot_api::SetQueuedCmdStartExec();
  return CheckCommunication(status);
}

void CheckCommunicationWithException(const std::string &called_from, const uint8_t status)
{
  if (!CheckCommunication(status))
  {
    std::string str;
    CommunicationStatus2String(status, str);
    throw std::runtime_error(called_from + ":" + str);
  }
  return;
}

bool CheckCommunication(const uint8_t status)
{
  return status == dobot_api::DobotCommunicate_NoError;
}

void CommunicationStatus2String(const uint8_t status, std::string &str)
{
  switch (status)
  {
    case dobot_api::DobotCommunicate_NoError:
      str = "OK";
      break;
    case dobot_api::DobotCommunicate_BufferFull:
      str = "BUFFER_FULL";
      break;
    case dobot_api::DobotCommunicate_Timeout:
      str = "TIMEOUT";
      break;
    case dobot_api::DobotCommunicate_InvalidParams:
      str = "INVALID_PARAM";
      break;
    default:
      str = "UNEXPECTED";
      break;
  }
  return;
}

void CheckConnectionWithException(const std::string &called_from, const uint8_t status)
{
  if (!CheckConnection(status))
  {
    std::string str;
    ConnectionStatus2String(status, str);
    throw std::runtime_error(called_from + ":" + str);
  }
  return;
}

bool CheckConnection(const uint8_t status)
{
  return status == dobot_api::DobotConnect_NoError;
}

void ConnectionStatus2String(const uint8_t status, std::string &str)
{
  switch (status)
  {
    case dobot_api::DobotConnect_NoError:
      str = "OK";
      break;
    case dobot_api::DobotConnect_NotFound:
      str = "NOT_FOUND";
      break;
    case dobot_api::DobotConnect_Occupied:
      str = "OCCUPIED";
      break;
    default:
      str = "UNEXPECTED";
      break;
  }
  return;
}

}  // namespace dobot_m1_interface