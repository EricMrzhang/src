#include <string>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;

namespace olelidar
{
   class HttpRequest
   {
    public:
       HttpRequest(const std::string& ip)
       {
          m_ip=ip;
          m_port=80;
       }
       

       // Http GET请求
       int HttpGet(std::string req,std::string &ret)
       {
          
          ret="";
          int socket_id=socket(AF_INET, SOCK_STREAM, 0);
          if (socket_id == -1) {
          cout<<"failed to open socket"<<endl;
          
          return 0;
          }
          sockaddr_in my_addr;                              // my address information
          memset(&my_addr, 0, sizeof(my_addr));
          my_addr.sin_addr.s_addr = inet_addr(m_ip.c_str());             // initialize to zeros
          my_addr.sin_family = AF_INET;                     // host byte order
          my_addr.sin_port = htons(uint16_t(m_port)); // short, in network byte order
          //my_addr.sin_addr.s_addr = INADDR_ANY;            // automatically fill in my IP
          int res = connect(socket_id,(struct sockaddr*)&my_addr,sizeof(my_addr));
          
          string strSend = "GET " + req + " HTTP/1.1\r\n"
                "Host: 192.168.1.100\r\n"
                "Accept-Encoding:gzip, deflate\r\n"
                "Accept-Language:zh-CN\r\n"
                "User-Agent:Mozilla/5.0\r\n\r\n";
          
          int errNo = send(socket_id, strSend.c_str(), strSend.length(), 0);
          
          if (errNo > 0)
            {
	            std::cout << "Sent successfully" << std::endl;
            }
            else
            {
                std::cout << "Failed to send"  << std::endl;
                return 0;
            }
          char bufRecv[1024] = { 0 };
          errNo = recv(socket_id, bufRecv, 1024, 0);
          if (errNo > 0)
            {
	            
                //int asciisize = ::WideCharToMultiByte(CP_OEMCP, 0, s.c_str(), -1, NULL, 0, NULL, NULL);
            	
                //errNo = recv(clientSocket, bufRecv, 3069, 0);
                ret = bufRecv;// 如果接收成功，则返回接收的数据内容
                
            }
            else
            {
                std::cout << "Failed to receive" << std::endl;
                return 0;
            }
             return 1;
       }

   

   

     private:
       std::string         m_ip;
       int             m_port;
   };
}


