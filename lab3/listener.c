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

#include "exlcm_message_t.h"

static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel, const exlcm_message_t *msg,
                       void *user)
{
    int i;
    printf("Received message on channel \"%s\":\n", channel);
    printf("  timestamp      = %" PRId64 "\n", msg->timestamp);
    printf("  id             = %d\n", msg->ID);
    printf("  name           = '%s'\n", msg->name);
    printf("  detectedObject = %d\n", msg->detectedObject);
    printf("  mode	     = %u\n", msg->mode);
}

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm)
        return 1;

    exlcm_message_t_subscribe(lcm, "MESSAGE", &my_handler, NULL);

    while (1)
        lcm_handle(lcm);

    lcm_destroy(lcm);
    return 0;
}
