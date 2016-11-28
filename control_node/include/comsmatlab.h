#ifndef COMSMATLAB_H
#define COMSMATLAB_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <QString>
#include "ros/ros.h"

#define BUFFER_SIZE 5120
#define PORT 12345
#define MACHINE_IP "172.16.49.15"

class ComsMatlab{
public:
   ComsMatlab();
   ~ComsMatlab();
   bool openComs();
   inline void closeComs() { close(socket_fd); }
   inline bool isOpen() { return active; }
   inline int writeDatagram(std::string datagram) { return sendto(socket_fd, datagram.c_str(), datagram.size(), 0, (struct sockaddr *)&destAddress, sock_addr_in_len); }
   inline int writeDatagram(QString datagram) { return writeDatagram(datagram.toStdString()); }
   std::string readDatagram();
   inline QString QReadDatagram() { return QString::fromStdString(readDatagram());}
private:
   bool active;
   int socket_fd;
   int recvlen;
   struct sockaddr_in destAddress;
   struct sockaddr sourceAddress;
   char buffer[BUFFER_SIZE];
   unsigned int sock_addr_in_len;
   unsigned int sock_addr_len;
};

#endif // BEHAVIOR_H
