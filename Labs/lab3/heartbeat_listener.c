// file: listener.c
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.c -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener listener.c `pkg-config --cflags --libs lcm`

#include <inttypes.h>
#include <lcm/lcm.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "exlcm_heartbeat_t.h"

int64_t get_timestamp_ns() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000000LL + tv.tv_usec * 1000LL;
}


static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel, const exlcm_heartbeat_t *msg,
                       void *user)
{
    int i;

    int64_t timestamp = get_timestamp_ns();
    printf("Received message on channel \"%s\":\n", channel);
    printf("  timestamp      = %" PRId64 "\n", msg->timestamp);
    printf("  id             = %d\n", msg->ID);
    printf("  received timestamp      = %" PRId64 "\n", timestamp);
    printf("  difference in ns      = %" PRId64 "\n", timestamp - msg->timestamp);
}

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm)
        return 1;

    exlcm_heartbeat_t_subscribe(lcm, "HEARTBEAT", &my_handler, NULL);

    while (1)
        lcm_handle(lcm);

    lcm_destroy(lcm);
    return 0;
}
