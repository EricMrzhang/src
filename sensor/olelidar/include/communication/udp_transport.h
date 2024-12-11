#pragma once

#include "communication/transport.h"

class UDPTransport : public Transport
{
public:
  UDPTransport(std::string address, std::string port = "0");

  ~UDPTransport();

  virtual bool connect();
  virtual bool disconnect();
  virtual bool read(boost::array<uint8_t, 4096>& buf, size_t& len);
  virtual bool readWithTimeout(boost::array<uint8_t, 4096>& buf, size_t& len, const uint32_t expiry_time);

private:
  std::unique_ptr<boost::asio::ip::udp::socket> socket_;
};
