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

void SetArmOrientationRight()
{
  uint8_t status = dobot_api::SetArmOrientation(dobot_api::RightyArmOrientation, false, nullptr);
  CheckCommunicationWithException("SetArmOrientationRight", status);
}

bool TrySetArmOrientationRight()
{
  uint8_t status = dobot_api::SetArmOrientation(dobot_api::RightyArmOrientation, false, nullptr);
  return CheckCommunication(status);
}

void SetArmOrientationLeft()
{
  uint8_t status = dobot_api::SetArmOrientation(dobot_api::LeftyArmOrientation, false, nullptr);
  CheckCommunicationWithException("SetArmOrientationLeft", status);
}

bool TrySetArmOrientationLeft()
{
  uint8_t status = dobot_api::SetArmOrientation(dobot_api::LeftyArmOrientation, false, nullptr);
  return CheckCommunication(status);
}

void SetPtpCmd(uint8_t mode, float x, float y, float z, float r)
{
  dobot_api::PTPCmd cmd;
  cmd.ptpMode = mode;
  cmd.x = x;
  cmd.y = y;
  cmd.z = z;
  cmd.r = r;

  uint8_t status = dobot_api::SetPTPCmd(&cmd, false, nullptr);
  CheckCommunicationWithException("SetPTPCmd", status);
}

bool TrySetPtpCmd(uint8_t mode, float x, float y, float z, float r)
{
  dobot_api::PTPCmd cmd;
  cmd.ptpMode = mode;
  cmd.x = x;
  cmd.y = y;
  cmd.z = z;
  cmd.r = r;

  uint8_t status = dobot_api::SetPTPCmd(&cmd, false, nullptr);
  return CheckCommunication(status);
}

void SetCpCmd(uint8_t mode, float x, float y, float z)
{
  dobot_api::CPCmd cmd;
  cmd.cpMode = mode;
  cmd.x = x;
  cmd.y = y;
  cmd.z = z;

  uint8_t status;
  status = dobot_api::SetCPCmd(&cmd, false, nullptr);
  CheckCommunicationWithException("SetCPcmd", status);
}

bool TrySetCpCmd(uint8_t mode, float x, float y, float z)
{
  dobot_api::CPCmd cmd;
  cmd.cpMode = mode;
  cmd.x = x;
  cmd.y = y;
  cmd.z = z;

  uint8_t status = dobot_api::SetCPCmd(&cmd, false, nullptr);
  return CheckCommunication(status);
}

bool TryIdleJog()
{
  dobot_api::JOGCmd jog_cmd;
  jog_cmd.isJoint = true;
  jog_cmd.cmd = dobot_api::JogIdle;
  uint8_t status = dobot_api::SetJOGCmd(&jog_cmd, true, nullptr);
  return CheckCommunication(status);
}

void WaitQueuedCmd(const uint64_t last_index)
{
  uint64_t current_index;
  while (1)
  {
    const uint8_t status = dobot_api::GetQueuedCmdCurrentIndex(&current_index);
    CheckCommunicationWithException("GetQueuedCmdCurrentIndex", status);

    if (last_index <= current_index)
      break;
  }
};

bool TryWaitQueuedCmd(const uint64_t last_index)
{
  uint64_t current_index;
  while (1)
  {
    const uint8_t status = dobot_api::GetQueuedCmdCurrentIndex(&current_index);
    if (!CheckCommunication(status))
      return false;

    if (last_index <= current_index)
      break;
  }
  return true;
}

void GetAlarmCode(uint32_t &code)
{
  dobot_api::alarmState alarm_state;
  uint32_t len;
  uint8_t state;
  state = dobot_api::GetAlarmsState(alarm_state.value, &len, 32);
  CheckCommunicationWithException("GetAlarmsState", state);
  code = dobot_api::AlarmStateToCode(alarm_state);
}

bool TryGetAlarmCode(uint32_t &code)
{
  dobot_api::alarmState alarm_state;
  uint32_t len;
  uint8_t state;
  state = dobot_api::GetAlarmsState(alarm_state.value, &len, 32);
  if (!CheckCommunication(state))
    return false;
  code = dobot_api::AlarmStateToCode(alarm_state);
  return true;
}

bool CheckAlarmCode(const uint32_t &code)
{
  return code == 0;
}

void AlarmCode2String(const uint32_t &code, std::string &str)
{
  str = "";
  str = dobot_api::GetAlarmsCodeName(dobot_api::AlarmCode(code));
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