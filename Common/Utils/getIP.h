//Network includes for getRobotIdByIP()
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
#include <ros/ros.h>

int getRobotIdByIP(std::string ifname)
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
	int id = if_info.ifr_hwaddr.sa_data[5];
   if(id<1 || id>6) id = -1;
   return id;
}
