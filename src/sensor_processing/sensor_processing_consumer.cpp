#include "sensor_processing_stage.h"
#include "processing_pipeline.h"

#include <map>

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_processing_consumer, LOG_LEVEL_WRN);

ZBUS_SUBSCRIBER_DEFINE(sensor_processing_sub, 16);
ZBUS_CHAN_DECLARE(sensor_chan);

ZBUS_CHAN_ADD_OBS(sensor_chan, sensor_processing_sub, 3);

#define PROC_STACK_SIZE  3072
#define PROC_THREAD_PRIO 5

K_THREAD_STACK_DEFINE(proc_stack, PROC_STACK_SIZE);
static struct k_thread proc_thread;

static std::map<const char*, std::unique_ptr<ProcessingPipeline>> processing_pipelines;

void set_processing_pipeline(const char *name, std::unique_ptr<ProcessingPipeline> pipeline) {
    processing_pipelines[name] = std::move(pipeline);
}

ProcessingPipeline* get_processing_pipeline(const char *name) {
    auto it = processing_pipelines.find(name);
    if (it != processing_pipelines.end()) {
        return it->second.get();
    }
    return nullptr;
}

void remove_processing_pipeline(const char *name) {
    processing_pipelines.erase(name);
}

/* Dedicated consumer that blocks on the subscriber queue */
static void processing_thread(void *a, void *b, void *c)
{
    const struct zbus_channel *chan;

    while (true) {
        /* Wait until a message for our subscriber is available */
        int err = zbus_sub_wait(&sensor_processing_sub, &chan, K_FOREVER);
        if (err) {
            LOG_WRN("zbus_sub_wait err=%d", err);
            continue;
        }

        /* Copy the message atomically out of the channel */
        struct sensor_msg msg;
        err = zbus_chan_read(chan, &msg, K_NO_WAIT);
        if (err) {
            LOG_WRN("zbus_chan_read err=%d", err);
            continue;
        }

        if (!(msg.consumer_mask & SENSOR_CONSUMER_PROCESSING)) {
            continue;
        }

        for (auto &[name, pipeline] : processing_pipelines) {
            if (pipeline) {
                LOG_DBG("Injecting sample into pipeline %s", name);
                // TODO: handle output
                pipeline->inject(msg.data);
            }
        }
    }
}

/* Bring up the dedicated thread */
int sensor_processing_consumer_init(void)
{
    k_thread_create(&proc_thread, proc_stack, K_THREAD_STACK_SIZEOF(proc_stack),
                    processing_thread, NULL, NULL, NULL,
                    PROC_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&proc_thread, "sensor_proc");
    return 0;
}