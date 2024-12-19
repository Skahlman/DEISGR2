#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <lcm/lcm.h>

#include "exlcm_heartbeat_t.h"  
 
#define HEARTBEAT_INTERVAL 1 
#define CHANNEL "HEARTBEAT"

typedef struct {
    int ID;
    lcm_t *lcm;
} entity_info_t;


int64_t get_timestamp_ns() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000000LL + tv.tv_usec * 1000LL;
}


void *send_heartbeat(void *arg) {
    entity_info_t *info = (entity_info_t *)arg;
    lcm_t *lcm = info->lcm;
    int id = info->ID;

    while (1) {
        exlcm_heartbeat_t msg;
        msg.timestamp = get_timestamp_ns();
        msg.ID = id;

        exlcm_heartbeat_t_publish(lcm, CHANNEL, &msg);

        printf("Entity %d sent heartbeat at timestamp %lld\n", id, (long long)msg.timestamp);

        sleep(HEARTBEAT_INTERVAL);
    }

    return NULL;
}

int main(int argc, char **argv) {
    lcm_t *lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm) {
        fprintf(stderr, "Error: Could not create LCM instance\n");
        return 1;
    }

    int num_entities;

    if (argc < 2)
        num_entities = 1;
    else
        num_entities = atoi(argv[1]);


    pthread_t threads[num_entities];
    entity_info_t entity_infos[num_entities];

    for (int i = 0; i < num_entities; i++) {
        entity_infos[i].ID = i;
        entity_infos[i].lcm = lcm;

        if (pthread_create(&threads[i], NULL, send_heartbeat, &entity_infos[i])) {
            fprintf(stderr, "Error: Could not create thread for entity %d\n", i);
            return 1;
        }
    }

    for (int i = 0; i < num_entities; i++) {
        pthread_join(threads[i], NULL);
    }

    lcm_destroy(lcm);
    return 0;
}