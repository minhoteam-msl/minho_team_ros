/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA COMM
 *
 * CAMBADA COMM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA COMM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>

#include "multicast.h"


#define PERRNO(txt) \
	printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

#define PERR(txt, par...) \
	printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)

#ifdef DEBUG
#define PDEBUG(txt, par...) \
	printf("DEBUG: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)
#else
#define PDEBUG(txt, par...)
#endif


struct sockaddr_in destAddress;


int if_NameToIndex(std::string ifname, char *address)
{
	int	fd;
	struct ifreq if_info;
	int if_index;

	memset(&if_info, 0, sizeof(if_info));
	strncpy(if_info.ifr_name, ifname.c_str(), IFNAMSIZ-1);

	if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		ROS_ERROR("Error getting UDP socket");
		return -1;
	}
	if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
	{
		ROS_ERROR("Error sending IOCTL 1");
		close(fd);
		return -1;
	}
	if_index = if_info.ifr_ifindex;

	if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
	{
		ROS_ERROR("Error sending IOCTL 2");
		close(fd);
		return -1;
	}
	
	close(fd);

	sprintf(address, "%d.%d.%d.%d",
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[2],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[3],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[4],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[5]);
	
	ROS_WARN("Using device %s : Agent IP -> %s", if_info.ifr_name, address);

	return if_index;
}


//	*************************
//  Open Socket
//
int openSocket(std::string interface, std::string *ip_base, uint8_t *agent_id,int recv_own_data)
{
   struct sockaddr_in multicastAddress;
   struct ip_mreqn mreqn;
   struct ip_mreq mreq;
	int multiSocket;
	int opt;
    char address[20];
	
   std::string mcastIP = MULTICAST_IP; // default value
   // red iptable.cfg from common
   std::string ipFilePath = getenv("HOME");
   std::string line = "";
   ipFilePath += "/Common/iptable.cfg";
   std::ifstream file; file.open(ipFilePath);
   if(file.is_open()){
      while (getline(file,line)){
          if(line.size()>0 && line[0]!='#'){
             if(line.find("MC")!=std::string::npos){
                mcastIP = line.substr(0,line.find(" "));
                ROS_INFO("Found Multicast IP config at iptable.cfg - %s",mcastIP.c_str());   
             }
          }
      }
      file.close();
   } else ROS_ERROR("Failed to read iptable.cfg");
   
   
   bzero(&multicastAddress, sizeof(struct sockaddr_in));
   multicastAddress.sin_family = AF_INET;
   multicastAddress.sin_port = htons(MULTICAST_PORT);
   multicastAddress.sin_addr.s_addr = INADDR_ANY;

	bzero(&destAddress, sizeof(struct sockaddr_in));
	destAddress.sin_family = AF_INET;
	destAddress.sin_port = htons(MULTICAST_PORT);
	destAddress.sin_addr.s_addr = inet_addr(mcastIP.c_str());

	if((multiSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		ROS_ERROR("Error creating Multicast UDP socket : %s",strerror(errno));
		return -1;
	}

	memset((void *) &mreqn, 0, sizeof(mreqn));
	mreqn.imr_ifindex=if_NameToIndex(interface, address);
	
	ip_base->assign(address,20);
	std::string temp = ip_base->substr(ip_base->find_last_of(".")+1,ip_base->size());
	(*ip_base) = ip_base->substr(0,ip_base->find_last_of("."));
	if(strcmp("172.16.49",ip_base->c_str())!=0) (*agent_id) = 0;
	else (*agent_id) = std::stoi(temp);
	
	ROS_INFO("Ip Base: %s | Agent ID: %d",ip_base->c_str(),*agent_id);
	if((setsockopt(multiSocket, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
	{
	   ROS_ERROR("Error setting socket option 1: %s",strerror(errno));
		return -1;
	}

	opt = 1;
	if((setsockopt(multiSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
    {
      ROS_ERROR("Error setting socket option 2: %s",strerror(errno));
		return -1;
    }
 
	memset((void *) &mreq, 0, sizeof(mreq));
	mreq.imr_multiaddr.s_addr = inet_addr(mcastIP.c_str());
	mreq.imr_interface.s_addr = inet_addr(address);

	if((setsockopt(multiSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
	{
	   ROS_ERROR("Error setting socket option 3: %s",strerror(errno));
		return -1;
	}
						
	/* Disable reception of our own multicast */
	opt = recv_own_data;
	if((setsockopt(multiSocket, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
	{
		ROS_ERROR("Error setting socket option 4: %s",strerror(errno));
		return -1;
	}

	if(bind(multiSocket, (struct sockaddr *) &multicastAddress, sizeof(struct sockaddr_in)) == -1)
	{
		ROS_ERROR("Error binding socket: %s",strerror(errno));
		return -1;
	}

	return (multiSocket);
}



//	*************************
//  Close Socket
//
void closeSocket(int multiSocket)
{
	if(multiSocket != -1)
		shutdown(multiSocket, SHUT_RDWR);
}



//	*************************
//  Send Data
//
int sendData(int multiSocket, void* data, int dataSize)
{
	return sendto(multiSocket, data, dataSize, 0, (struct sockaddr *)&destAddress, sizeof (struct sockaddr));
}



//	*************************
//  Receive Data
//
int receiveData(int multiSocket, void* buffer, int bufferSize)
{
	return recv(multiSocket, buffer, bufferSize, 0);
}
