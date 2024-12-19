// file: send_message.c
//
// LCM example program.
//
// compile with:
//  $ gcc -o send_message send_message.c -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o send_message send_message.c `pkg-config --cflags --libs lcm`

#include <lcm/lcm.h>
#include <stdio.h>
#include <sys/time.h>

#include "exlcm_message_t.h"

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
        return 1;

    struct timeval start;
    gettimeofday(&start, NULL);

    exlcm_message_t my_data = {
        .timestamp = (start.tv_sec * 1000) + (start.tv_usec / 1000),
        .ID = 0,
        .name = "test",
        .detectedObject = 1,
        .mode = 2
    };

    exlcm_message_t_publish(lcm, "MESSAGE", &my_data);

    lcm_destroy(lcm);
    return 0;
}
