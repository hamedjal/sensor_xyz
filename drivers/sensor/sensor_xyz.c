/*
 * Fake 3-axis accelerometer.
 */

#define DT_DRV_COMPAT sensor_xyz

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/spinlock.h>

#include "sensor_xyz.h"

LOG_MODULE_REGISTER(sensor_xyz, CONFIG_SENSOR_XYZ_LOG_LEVEL);

#define XYZ_PERIOD_US      (USEC_PER_SEC / CONFIG_SENSOR_XYZ_RATE_HZ)
#define XYZ_GRAVITY_UMS2   9806650LL   
#define XYZ_AMPLITUDE_UMS2 2000000.0f 
#define XYZ_WAVE_PERIOD_MS 2000  
#define XYZ_TWO_PI         6.28318530718f

struct xyz_data {
	struct k_timer timer;
	struct k_work work;
	struct k_spinlock lock;

	struct sensor_value latest[3];   
	int64_t latest_ts;
	bool have_sample;

	struct sensor_value latched[3];  
	int64_t latched_ts;
};

static void micro_to_sv(int64_t micro, struct sensor_value *v)
{
	v->val1 = (int32_t)(micro / 1000000);
	v->val2 = (int32_t)(micro % 1000000);
}

static void xyz_work_handler(struct k_work *work)
{
	struct xyz_data *data = CONTAINER_OF(work, struct xyz_data, work);
	int64_t now = k_uptime_get();
	float phase = (float)(now % XYZ_WAVE_PERIOD_MS) *
		      (XYZ_TWO_PI / XYZ_WAVE_PERIOD_MS);

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	micro_to_sv((int64_t)(sinf(phase) * XYZ_AMPLITUDE_UMS2), &data->latest[0]);
	micro_to_sv((int64_t)(cosf(phase) * XYZ_AMPLITUDE_UMS2), &data->latest[1]);
	micro_to_sv(XYZ_GRAVITY_UMS2, &data->latest[2]);
	data->latest_ts = now;
	data->have_sample = true;

	k_spin_unlock(&data->lock, key);
}

static void xyz_timer_handler(struct k_timer *timer)
{
	struct xyz_data *data = CONTAINER_OF(timer, struct xyz_data, timer);

	k_work_submit(&data->work);
}

static int xyz_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct xyz_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		break;
	default:
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (!data->have_sample) {
		k_spin_unlock(&data->lock, key);
		return -ENODATA;
	}
	memcpy(data->latched, data->latest, sizeof(data->latched));
	data->latched_ts = data->latest_ts;

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int xyz_channel_get(const struct device *dev, enum sensor_channel chan,
			   struct sensor_value *val)
{
	struct xyz_data *data = dev->data;
	k_spinlock_key_t key;

	if (val == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:     
		val[0] = data->latched[0];
		val[1] = data->latched[1];
		val[2] = data->latched[2];
		break;
	case SENSOR_CHAN_ACCEL_X:
		val[0] = data->latched[0];
		break;
	case SENSOR_CHAN_ACCEL_Y:
		val[0] = data->latched[1];
		break;
	case SENSOR_CHAN_ACCEL_Z:
		val[0] = data->latched[2];
		break;
	default:
		k_spin_unlock(&data->lock, key);
		return -ENOTSUP;
	}
	k_spin_unlock(&data->lock, key);
	return 0;
}

int64_t sensor_xyz_get_timestamp(const struct device *dev)
{
	struct xyz_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int64_t ts = data->latched_ts;

	k_spin_unlock(&data->lock, key);
	return ts;
}

static void xyz_start(const struct device *dev)
{
	struct xyz_data *data = dev->data;

	k_work_submit(&data->work);  /* first sample immediately */
	k_timer_start(&data->timer, K_USEC(XYZ_PERIOD_US), K_USEC(XYZ_PERIOD_US));
}

static void xyz_stop(const struct device *dev)
{
	struct xyz_data *data = dev->data;
	struct k_work_sync sync;

	k_timer_stop(&data->timer);
	k_work_cancel_sync(&data->work, &sync);
}

static int xyz_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		xyz_stop(dev);
		return 0;
	case PM_DEVICE_ACTION_RESUME:
		xyz_start(dev);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int xyz_init(const struct device *dev)
{
	struct xyz_data *data = dev->data;

	k_work_init(&data->work, xyz_work_handler);
	k_timer_init(&data->timer, xyz_timer_handler, NULL);
	xyz_start(dev);

	LOG_INF("%s: fake accel running at %d Hz", dev->name,
		CONFIG_SENSOR_XYZ_RATE_HZ);
	return 0;
}

static const struct sensor_driver_api xyz_api = {
	.sample_fetch = xyz_sample_fetch,
	.channel_get = xyz_channel_get,
};

#define XYZ_INIT(inst)                                                             \
	BUILD_ASSERT(DT_INST_PROP(inst, odr_hz) == CONFIG_SENSOR_XYZ_RATE_HZ,      \
		     "odr-hz must equal CONFIG_SENSOR_XYZ_RATE_HZ");               \
	static struct xyz_data xyz_data_##inst;                                    \
	PM_DEVICE_DT_INST_DEFINE(inst, xyz_pm_action);                             \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, xyz_init, PM_DEVICE_DT_INST_GET(inst),  \
				     &xyz_data_##inst, NULL, POST_KERNEL,          \
				     CONFIG_SENSOR_INIT_PRIORITY, &xyz_api);

DT_INST_FOREACH_STATUS_OKAY(XYZ_INIT)