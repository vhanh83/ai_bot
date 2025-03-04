#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
using namespace std;

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
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
    serial_conn_.SetDTR(true);
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
      std::string response = "";
      serial_conn_.FlushIOBuffers(); // Just in case
      serial_conn_.Write(msg_to_send); 
      try
      {
        // Responses end with \r\n so we will read up to (and including) the \n.
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
      }
      catch (const LibSerial::ReadTimeout&)
      {
          std::cerr << "The ReadByte() call has timed out." << std::endl ;
      }
      
      if (print_output)
      {
        std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
      }

      return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }


  std::vector<std::string> SplitString(
      std::string str,
      std::string delimeter)
  {
      std::vector<std::string> splittedStrings = {};
      size_t pos = 0;

      while ((pos = str.find(delimeter)) != std::string::npos)
      {
          std::string token = str.substr(0, pos);
          if (token.length() > 0)
              splittedStrings.push_back(token);
          str.erase(0, pos + delimeter.length());
      }

      if (str.length() > 0)
          splittedStrings.push_back(str);
      return splittedStrings;
  }
  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string responseEn1 = send_msg("$GE#");//std::cout << "E1" << std::endl;
    //std::string responseEn2 = send_msg("$GE;2;0;0#");//std::cout << "E2" << std::endl;

    std::vector<std::string> resultsEnc1;

    //std::vector<std::string> resultsEnc2;

    boost::split(resultsEnc1, responseEn1, [](char c){return c == ';';});

   // boost::split(resultsEnc2, responseEn2, [](char c){return c == ';';});

    // }
    if (std::strcmp(resultsEnc1[0].c_str(),"$GE") == 0) //&& (std::atoi(resultsEnc1[1].c_str())==1)))
     {             
        boost::erase_all(resultsEnc1[1], "#");
        val_1 = std::atoi(resultsEnc1[1].c_str());
        boost::erase_all(resultsEnc1[2], "#");
        val_2 = std::atoi(resultsEnc1[2].c_str());
        //std::cout << "enc1: " << val_1 << std::endl;
     }
     
    // if ((std::strcmp(resultsEnc2[0].c_str(),"$GE") == 0 && (std::atoi(resultsEnc2[1].c_str())==2)))
  //  {
       // boost::erase_all(resultsEnc2[2], "#");
       // val_2 = std::atoi(resultsEnc2[2].c_str());
        //std::cout << "enc2: " << val_2 << std::endl;
    // }  
  }

  int prvval1 = 0, prvval2 = 0, count= 0;
  
  bool isCount= false; 
  int countSendLimit = 5;     //so lan gui
  void set_motor_values(int val1, int val2)
  {
      if(val1 != prvval1 || val2 != prvval2)
      {
        isCount = true; count = 0;
      }
      if(isCount)
      {
        count++;
      }
      if(count > 0 && count <= countSendLimit)
      {
        std::stringstream ss1;
        ss1 << "$SR;" << 2*val1 <<";"<< 2*val2<<"#";
        //std::cout << "M" << std::endl;
        send_msg(ss1.str());
        //std::cout << "C: " << count << std::endl;
      }
      else {isCount = false; count = 0;}

      prvval1 = val1; prvval2 = val2;
   
     
  }

  void set_pid_values(float k_p, float k_d, float k_i, float k_o)
  {
    std::stringstream ss;

    ss << "$PI;"<< k_p << ";"<< k_d << ";" << k_i << ";" << k_o << "#";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP