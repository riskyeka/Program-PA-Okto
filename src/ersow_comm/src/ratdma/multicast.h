#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <linux/if.h>
#include <linux/if_ether.h>

#define MULTICAST_IP	"224.16.32.33"
#define MULTICAST_PORT	50000
#define TTL				64
#define RECEIVE_OUR_DATA 0


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

int if_NameToIndex(char* ifname, char* address)
{
	int fd;
	struct ifreq if_info;
	int if_index;

	memset(&if_info, 0, sizeof(if_info));
	strncpy(if_info.ifr_name, ifname, IFNAMSIZ-1);

	if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		PERRNO("socket");
		return (-1);
	}
	if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
	{
		PERRNO("ioctl");
		close(fd);
		return (-1);
	}
	if_index = if_info.ifr_ifindex;

	if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
	{
		PERRNO("ioctl");
		close(fd);
		return (-1);
	}
	
	close(fd);

	sprintf(address, "%d.%d.%d.%d\n",
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[2],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[3],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[4],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[5]);

	printf("**** Using device %s -> Ethernet %s\n ****", if_info.ifr_name, address);

	return (if_index);

}

int openSocket(char* interface)
{
	struct sockaddr_in multicastAddress;
	struct ip_mreqn mreqn;
	struct ip_mreq mreq;
	int multisocket;
	char address[20];
	int opt;

	bzero(&multicastAddress, sizeof(struct sockaddr_in));
	multicastAddress.sin_family = AF_INET;
	multicastAddress.sin_port = htons(MULTICAST_PORT);
	multicastAddress.sin_addr.s_addr = INADDR_ANY;

	bzero(&destAddress, sizeof(struct sockaddr_in));
	destAddress.sin_family = AF_INET;
	destAddress.sin_port = htons(MULTICAST_PORT);
	destAddress.sin_addr.s_addr = inet_addr(MULTICAST_IP);

	// printf("Creating socket ..\n");
	// multisocket = ;
	if((multisocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		PERRNO("socket");
		return (-1);
	} 

	// join multicast group
	memset((void *) &mreqn, 0, sizeof(mreqn));
	mreqn.imr_ifindex = if_NameToIndex(interface, address);	
	if ((setsockopt(multisocket, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
	{
		PERRNO("setsockopt");
		return (-1);
	}

	opt = 1;
	if((setsockopt(multisocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
  	{
    	PERRNO("setsockopt");
		return (-1);
  	}
	

	// This option is used to join a multicast group on a specific interface  
	memset((void *) &mreq, 0, sizeof(mreq));
	mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_IP);
	mreq.imr_interface.s_addr = inet_addr(address);

	if((setsockopt(multisocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
	{
	  	PERRNO("setsockopt");
		return (-1);
	}
						
	/* Disable reception of our own multicast */
	opt = RECEIVE_OUR_DATA;
	if((setsockopt(multisocket, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
	{
		PERRNO("setsockopt");
		return (-1);
	}

	if(bind(multisocket, (struct sockaddr *) &multicastAddress, sizeof(struct sockaddr_in)) == -1)
	{
		PERRNO("bind");
		return (-1);
	}

	// printf("multisocket = %d\n", multisocket);
	return (multisocket);
}


int closeSocket(int multiSocket)
{
	if(multiSocket != -1)
		shutdown(multiSocket, SHUT_RDWR);
}

int sendData(int multiSocket, void* data, int dataSize)
{
	return (sendto(multiSocket, data, dataSize, 0, (struct sockaddr *)&destAddress, sizeof(struct sockaddr)));
}

int receiveData(int multiSocket, void *buffer, int bufferSize) 
{
	return (recv(multiSocket, buffer, bufferSize, 0));
}