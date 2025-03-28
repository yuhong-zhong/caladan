#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <base/stddef.h>
#include <base/bitmap.h>
#include <base/log.h>
#include <base/cpu.h>
#include <base/mem.h>
#include <net/ip.h>

#include <iokernel/control.h>

static int str_to_ip(const char *str, uint32_t *addr)
{
	uint8_t a, b, c, d;
	if(sscanf(str, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d) != 4) {
		return -EINVAL;
	}

	*addr = MAKE_IP_ADDR(a, b, c, d);
	return 0;
}

int main(int argc, char **argv)
{
        if (argc != 3) {
                log_err("Usage: %s <socket_index> <ip>", argv[0]);
                return -1;
        }

        int socket_index = atoi(argv[1]);
        RT_BUG_ON(socket_index < 0);

        uint32_t ip;
        if (str_to_ip(argv[2], &ip) != 0) {
                log_err("Invalid IP address: %s", argv[2]);
                return -1;
        }

	struct sockaddr_un addr;
	int socket_command = IOK_FAILOVER;
	ssize_t ret;
	int fd;

	// Make sure it's an abstract namespace path.
	assert(CONTROL_SOCK_PATH_PREFIX[0] == '\0');

	BUILD_ASSERT(sizeof(CONTROL_SOCK_PATH_PREFIX) <= sizeof(addr.sun_path));
	addr.sun_family = AF_UNIX;
	memcpy(addr.sun_path, CONTROL_SOCK_PATH_PREFIX, sizeof(CONTROL_SOCK_PATH_PREFIX));
	snprintf(addr.sun_path + sizeof(CONTROL_SOCK_PATH_PREFIX) - 1,
		 sizeof(addr.sun_path) - sizeof(CONTROL_SOCK_PATH_PREFIX) - 1,
		 "%d", socket_index);

	log_info("host_agent: using socket path %s", addr.sun_path + 1);

	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd == -1) {
		log_err("host_agent: socket() failed [%s]", strerror(errno));
		RT_BUG_ON(true);
	}

	if (connect(fd, (struct sockaddr *)&addr,
		    sizeof(addr.sun_family) + strlen(addr.sun_path + 1) + 2) == -1) {
		log_err("host_agent: connect() failed [%s]", strerror(errno));
		RT_BUG_ON(true);
	}

	ret = write(fd, &socket_command, sizeof(socket_command));
	if (ret != sizeof(socket_command)) {
		log_err("host_agent: write(socket_command) failed, len=%ld [%s]",
			ret, strerror(errno));
		RT_BUG_ON(true);
	}

	ret = write(fd, &ip, sizeof(ip));
	if (ret != sizeof(ip)) {
		log_err("host_agent: write(ip) failed, len=%ld [%s]",
			ret, strerror(errno));
		RT_BUG_ON(true);
	}

	close(fd);

	return 0;
}
