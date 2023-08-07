#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <bitset>
#include <cstddef>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cstring>

#include <zlib.h>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    uLong crc = crc32(0L, Z_NULL, 0);
    serial_conn_.FlushIOBuffers(); // Just in case

    std::byte bytes[msg_to_send.length()];
    std::memcpy(bytes, msg_to_send.data(), msg_to_send.length());

    crc = crc32(crc, (const Bytef *)bytes, sizeof(bytes));
    std::string msg_to_serial = std::to_string(crc) + msg_to_send + "\r\n";
    serial_conn_.Write(msg_to_serial);

    std::string response = "";
    // try
    // {
    //   // Responses end with \r\n so we will read up to (and including) the \n.
    //   serial_conn_.ReadLine(response, '\n', timeout_ms_);
    // }
    // catch (const LibSerial::ReadTimeout &)
    // {
    //   std::cerr << "The ReadByte() call has timed out." << std::endl;
    // }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_serial << " Recv: " << response << std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    std::string response = send_msg("");
  }

  void control_cmd_generate(char *cmd, double front_right_vel, double rear_right_vel, double rear_left_vel, double front_left_vel)
  {
    sprintf(cmd, "{\"topic\":\"wheel_control\",\"velocity\":[%.2f,%.2f,%.2f,%.2f]}\n", front_right_vel, rear_right_vel, rear_left_vel, front_left_vel);
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }
  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "{\"topic\":\"wheel_control\",\"velocity\":[" << val_1 << "," << val_2 << "]}";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o;
    send_msg(ss.str());
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP