#include "comsmatlab.h"

ComsMatlab::ComsMatlab()
{
   active = false;   
   sock_addr_in_len =  sizeof(destAddress);
   sock_addr_len = sizeof(sourceAddress);
}

ComsMatlab::~ComsMatlab()
{
   if(active) close(socket_fd);  
}

bool ComsMatlab::openComs()
{
   if((socket_fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))<0){
      ROS_ERROR("Failed to create Matlab's UDP bridge.");
      active = false;
      return active;
   }
   
   memset((char *) &destAddress, 0, sock_addr_in_len);
   destAddress.sin_family = AF_INET;
   destAddress.sin_port = htons(PORT);
   if(inet_aton(MACHINE_IP, &destAddress.sin_addr)==0) {
      ROS_ERROR("Failed to set target machine ip - inet_aton()");
      active = false;
      return active;
   }
   
   ROS_INFO("Matlab's UDP bridge running.");
   active = true;
   return active;
}

std::string ComsMatlab::readDatagram()
{
   std::string datagram = "";
   recvlen = recvfrom(socket_fd, buffer, BUFFER_SIZE, 0, &sourceAddress, &sock_addr_len);
   datagram.assign(buffer,recvlen);
   return datagram;
}
