/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "util.h"
#include "connection/connection.h"
#include "calibration.h"

#include <math.h>

#include "fusion/fusions.h"
#include "sensors.h"

#include "sensor.h"

#if DT_NODE_EXISTS(DT_NODELABEL(imu_spi))
#define SENSOR_IMU_SPI_EXISTS true
#define SENSOR_IMU_SPI_NODE DT_NODELABEL(imu_spi)
#define SPI_OP SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8)
static struct spi_dt_spec sensor_imu_spi_dev = SPI_DT_SPEC_GET(SENSOR_IMU_SPI_NODE, SPI_OP, 0);
#else
static struct spi_dt_spec sensor_imu_spi_dev = {0};
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(imu))
#define SENSOR_IMU_EXISTS true
#define SENSOR_IMU_NODE DT_NODELABEL(imu)
static struct i2c_dt_spec sensor_imu_dev = I2C_DT_SPEC_GET(SENSOR_IMU_NODE);
#else
static struct i2c_dt_spec sensor_imu_dev = {0};
#endif
#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS
#error "IMU node does not exist"
#endif
static uint8_t sensor_imu_dev_reg = 0xFF;

#if DT_NODE_EXISTS(DT_NODELABEL(mag))
#define SENSOR_MAG_EXISTS true
#define SENSOR_MAG_NODE DT_NODELABEL(mag)
static struct i2c_dt_spec sensor_mag_dev = I2C_DT_SPEC_GET(SENSOR_MAG_NODE);
#else
#warning "Magnetometer node does not exist"
static struct i2c_dt_spec sensor_mag_dev = {0};
#endif
static uint8_t sensor_mag_dev_reg = 0xFF;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

static float q3[4] = {SENSOR_QUATERNION_CORRECTION}; // correction quaternion
#if CONFIG_SELECT_DEVICE_ORIENTATION
static uint8_t device_orientation_idx = 0x00;
static float device_orientation[] =
{
	SENSOR_QUATERNION_CORRECTION,  // 0 = default
	0.0f, 0.0f, 0.0f, 1.0f,        // 1
	0.0f, 0.7071f, 0.0f, 0.7071f,  // 2
	0.0f, 1.0f, 0.0f, 0.0f,        // 3
	0.0f, -0.7071f, 0.0f, 0.7071f, // 4
	0.0f, 0.0f, 0.7071f, 0.7071f,  // 5
	0.5f, 0.5f, 0.5f, 0.5f,        // 6
	0.7071f, 0.7071f, 0.0f, 0.0f,  // 7
	-0.5f, -0.5f, 0.5f, 0.5f,      // 8
	0.0f, 0.f, -0.7071f, 0.7071f,  // 9
	-0.5f, 0.5f, -0.5f, 0.5f,      // 10
	-0.7071f, 0.7071f, 0.0f, 0.0f, // 11
	0.5f, -0.5f, -0.5f, 0.5f,      // 12
	0.7071f, 0.0f, 0.f, 0.7071f,   // 13
	0.5f, 0.5f, -0.5f, 0.5f,       // 14
	0.0f, 0.7071f, -0.7071f, 0.0f, // 15
	0.5f, -0.5f, 0.5f, 0.5f,       // 16
	1.0f, 0.0f, 0.0f, 0.0f,        // 17
	0.7071f, 0.0f, -0.7071f, 0.0f, // 18
	0.0f, 0.0f, 1.0f, 0.0f,        // 19
	0.7071f, 0.0f, 0.7071f, 0.0f,  // 20
	-0.7071f, 0.0f, 0.0f, 0.7071f, // 21
	-0.5f, 0.5f, 0.5f, 0.5f,       // 22
	0.0f, 0.7071f, 0.7071f, 0.0f,  // 23
	-0.5f, -0.5f, -0.5f, 0.5f      // 24
};
#endif

static float last_lin_a[3] = {0}; // vector to hold last linear accelerometer

static int64_t last_suspend_attempt_time = 0;
static int64_t last_data_time;
static int64_t last_info_time;
static int64_t last_mag_time;

static float max_gyro_speed_square;
static bool mag_use_oneshot;
static bool mag_skip_oneshot;

static float accel_actual_time;
static float gyro_actual_time;
static float mag_actual_time;

static bool sensor_fusion_init;
static bool sensor_sensor_init;

static bool sensor_sensor_scanning;

static bool main_suspended;

static bool mag_available;
#if MAG_ENABLED
static bool mag_enabled = true; // TODO: toggle from server
#else
static bool mag_enabled = false;
#endif

#if CONFIG_SENSOR_USE_XIOFUSION
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_fusion; // TODO: change from server
int fusion_id = FUSION_FUSION;
#elif CONFIG_SENSOR_USE_NXPSENSORFUSION
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_motionsense; // TODO: change from server
int fusion_id = FUSION_MOTIONSENSE;
#elif CONFIG_SENSOR_USE_VQF
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_vqf; // TODO: change from server
int fusion_id = FUSION_VQF;
#endif

static int sensor_imu_id = -1;
static int sensor_mag_id = -1;
static const sensor_imu_t *sensor_imu = &sensor_imu_none;
static const sensor_mag_t *sensor_mag = &sensor_mag_none;
static bool use_ext_fifo = false;

//#define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_INF);
#endif

K_THREAD_DEFINE(main_imu_thread_id, 1024, main_imu_thread, NULL, NULL, NULL, 7, 0, 0);

const char *sensor_get_sensor_imu_name(void)
{
	if (sensor_imu_id < 0)
		return "None";
	return dev_imu_names[sensor_imu_id];
}

const char *sensor_get_sensor_mag_name(void)
{
	if (sensor_mag_id < 0)
		return "None";
	return dev_mag_names[sensor_mag_id];
}

const char *sensor_get_sensor_fusion_name(void)
{
	if (fusion_id < 0)
		return "None";
	return fusion_names[fusion_id];
}

int sensor_init(void)
{
	while (sensor_sensor_scanning)
		k_usleep(1); // already scanning
	if (sensor_sensor_init)
		return 0; // already initialized
	sensor_sensor_scanning = true;

	sensor_scan_read();
	int imu_id = -1;
#if SENSOR_IMU_SPI_EXISTS
	// for SPI scan, set frequency of 10MHz, it will be set later by the driver initialization if needed
	sensor_imu_spi_dev.config.frequency = MHZ(10);
	LOG_INF("Scanning SPI bus for IMU");
	imu_id = sensor_scan_imu_spi(&sensor_imu_spi_dev, &sensor_imu_dev_reg);
	if (imu_id >= 0)
		sensor_interface_register_sensor_imu_spi(&sensor_imu_spi_dev);
#endif
#if SENSOR_IMU_EXISTS
	if (imu_id < 0)
	{
		LOG_INF("Scanning I2C bus for IMU");
		imu_id = sensor_scan_imu(&sensor_imu_dev, &sensor_imu_dev_reg);
		if (imu_id >= 0)
			sensor_interface_register_sensor_imu_i2c(&sensor_imu_dev);
	}
#else
	LOG_ERR("IMU node does not exist");
#endif
	if (imu_id >= (int)ARRAY_SIZE(dev_imu_names))
		LOG_WRN("Found unknown device");
	else if (imu_id < 0)
		LOG_ERR("No IMU detected");
	else
		LOG_INF("Found %s", dev_imu_names[imu_id]);
	if (imu_id >= 0)
	{
		if (imu_id >= (int)ARRAY_SIZE(sensor_imus) || sensor_imus[imu_id] == NULL || sensor_imus[imu_id] == &sensor_imu_none)
		{
			sensor_imu = &sensor_imu_none;
			sensor_sensor_scanning = false; // done
//			if (sensor_imu_dev.addr < 0xFF) // If for some reason there actually is a valid IMU but we found some unsupported device first
//			{
//				LOG_WRN("IMU not supported");
//				sensor_imu_dev.addr++;
//				sensor_imu_dev_reg = 0xFF;
//				sensor_scan_clear(); // clear the invalid data
//				return sensor_init(); // try again
//			}
			LOG_ERR("IMU not supported");
			set_status(SYS_STATUS_SENSOR_ERROR, true);
			return -1; // an IMU was detected but not supported
		}
		else
		{
			sensor_imu = sensor_imus[imu_id];
		}
	}
	else
	{
		sensor_imu = &sensor_imu_none;
		sensor_sensor_scanning = false; // done
		set_status(SYS_STATUS_SENSOR_ERROR, true);
		return -1; // no IMU detected! something is very wrong
	}

#if SENSOR_MAG_EXISTS
	LOG_INF("Scanning bus for magnetometer");
	int mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
	if (mag_id < 0)
	{
		// IMU must support passthrough mode if the magnetometer is connected through the IMU
		int err = sensor_imu->ext_passthrough(true);
		if (!err)
		{
			LOG_INF("Scanning bus for magnetometer through IMU passthrough");
			if (sensor_mag_dev.addr > 0x80) // marked as passthrough
			{
				sensor_mag_dev.addr &= 0x7F;
			}
			else
			{
				sensor_mag_dev.addr = 0x00; // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
			if (mag_id >= 0)
			{
				sensor_mag_dev.addr |= 0x80; // mark as passthrough
				use_ext_fifo = true;
			}
		}
		// sensor_imu->ext_passthrough(false);
	}
	else
	{
		use_ext_fifo = false;
	}
#else
	LOG_WRN("Magnetometer node does not exist");
	int mag_id = -1;
#endif
	if (mag_id >= (int)ARRAY_SIZE(dev_mag_names))
		LOG_WRN("Found unknown device");
	else if (mag_id < 0)
		LOG_WRN("No magnetometer detected");
	else
		LOG_INF("Found %s", dev_mag_names[mag_id]);
	if (mag_id >= 0) // if there is no magnetometer we do not care as much
	{
		if (mag_id >= (int)ARRAY_SIZE(sensor_mags) || sensor_mags[mag_id] == NULL || sensor_mags[mag_id] == &sensor_mag_none)
		{
			sensor_mag = &sensor_mag_none; 
			mag_available = false;
//			if (sensor_imu_dev.addr < 0xFF) // If for some reason there actually is a valid magnetometer but we found some unsupported device first
//			{
//				LOG_WRN("Magnetometer not supported");
//				sensor_mag_dev.addr++;
//				sensor_mag_dev_reg = 0xFF;
//				sensor_scan_clear(); // clear the invalid data
//				return sensor_init(); // try again
//			}
			LOG_ERR("Magnetometer not supported");
		}
		else
		{
			sensor_mag = sensor_mags[mag_id];
			mag_available = true;
		}
	}
	else
	{
		sensor_mag = &sensor_mag_none; 
		mag_available = false; // marked as not available
	}
	if (use_ext_fifo)
	{
		int err = mag_ext_setup(sensor_imu, sensor_mag, sensor_mag_dev.addr);
		if (err)
		{
			LOG_ERR("Magnetometer not supported by external interface");
			sensor_mag = &sensor_mag_none;
			mag_available = false;
		}
		else
		{
			sensor_mag = &sensor_mag_ext;
			mag_available = true;
		}
		
	}

	sensor_scan_write();
	sensor_interface_register_sensor_mag_i2c(&sensor_mag_dev); // TODO:
	connection_update_sensor_ids(imu_id, mag_id);
	sensor_imu_id = imu_id;
	sensor_mag_id = mag_id;

	sensor_sensor_init = true; // successfully initialized
	sensor_sensor_scanning = false; // done
	set_status(SYS_STATUS_SENSOR_ERROR, false); // clear error
	return 0;
}

void sensor_scan_read(void) // TODO: move some of this to sys?
{
	if (retained->imu_addr != 0)
	{
		sensor_imu_dev.addr = retained->imu_addr;
		sensor_imu_dev_reg = retained->imu_reg;
	}
	if (retained->mag_addr != 0)
	{
		sensor_mag_dev.addr = retained->mag_addr;
		sensor_mag_dev_reg = retained->mag_reg;
	}
	LOG_INF("IMU address: 0x%02X, register: 0x%02X", sensor_imu_dev.addr, sensor_imu_dev_reg);
	LOG_INF("Magnetometer address: 0x%02X, register: 0x%02X", sensor_mag_dev.addr, sensor_mag_dev_reg);
}

void sensor_scan_write(void) // TODO: move some of this to sys?
{
	retained->imu_addr = sensor_imu_dev.addr;
	retained->mag_addr = sensor_mag_dev.addr;
	retained->imu_reg = sensor_imu_dev_reg;
	retained->mag_reg = sensor_mag_dev_reg;
	retained_update();
}

void sensor_scan_clear(void) // TODO: move some of this to sys?
{
	retained->imu_addr = 0x00;
	retained->mag_addr = 0x00;
	retained->imu_reg = 0xFF;
	retained->mag_reg = 0xFF;
	retained_update();
}

void sensor_retained_read(void) // TODO: move some of this to sys? or move to calibration?
{
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	LOG_INF("Accelerometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->accBAinv[0][i], (double)retained->accBAinv[1][i], (double)retained->accBAinv[2][i], (double)retained->accBAinv[3][i]);
#else
	LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)retained->accelBias[0], (double)retained->accelBias[1], (double)retained->accelBias[2]);
#endif
	LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)retained->gyroBias[0], (double)retained->gyroBias[1], (double)retained->gyroBias[2]);
	if (mag_available && mag_enabled)
	{
//		LOG_INF("Magnetometer bridge offset: %.5f %.5f %.5f", (double)retained->magBias[0], (double)retained->magBias[1], (double)retained->magBias[2]);
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++)
			LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->magBAinv[0][i], (double)retained->magBAinv[1][i], (double)retained->magBAinv[2][i], (double)retained->magBAinv[3][i]);
	}
	if (retained->fusion_id)
		LOG_INF("Fusion data recovered");
}

void sensor_retained_write(void) // TODO: move to sys?
{
	if (!sensor_fusion_init)
		return;
//	memcpy(retained->magBias, sensor_calibration_get_magBias(), sizeof(retained->magBias));
	sensor_fusion->save(retained->fusion_data);
	retained->fusion_id = fusion_id;
	retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
	sys_interface_resume();
	int err = sensor_init(); // try initialization if possible
	if (mag_available) // try to shutdown magnetometer first (in case of passthrough)
		sensor_mag->shutdown();
	if (!err)
		sensor_imu->shutdown();
	else
		LOG_ERR("Failed to shutdown sensors");
	sys_interface_suspend();
}

uint8_t sensor_setup_WOM(void)
{
	sys_interface_resume();
	int err = sensor_init(); // try initialization if possible
	if (!err)
		return sensor_imu->setup_WOM();
	sys_interface_suspend(); // TODO: not suspending after WOM setup
	LOG_ERR("Failed to configure IMU wake up");
	return 0;
}

void sensor_fusion_invalidate(void)
{
	// TODO: reinitialize fusion
	if (sensor_fusion_init)
	{ // clear fusion gyro offset
		float g_off[3] = {0};
		sensor_fusion->set_gyro_bias(g_off);
		sensor_retained_write();
	}
	else
	{ // TODO: always clearing the fusion?
		retained->fusion_id = 0; // Invalidate retained fusion data
		retained_update();
	}
}

#if CONFIG_SELECT_DEVICE_ORIENTATION
void next_device_orientation(void)
{
	const size_t orientation_cnt = ARRAY_SIZE(device_orientation) / 4;
	if(++device_orientation_idx == orientation_cnt)
		device_orientation_idx = 0;
	set_device_orientation(device_orientation_idx);
}

void set_device_orientation(size_t n)
{
	const size_t max = ARRAY_SIZE(device_orientation) / 4 - 1;
	if(n > max)
	{
		printk("Expected a number between 0 and %d\n", max);
		return;
	}
	device_orientation_idx = n;
	memcpy(&q3[0], &device_orientation[4 * n], 4 * sizeof(float));
	printk("Orientation quat: %f %f %f %f\n", q3[0], q3[1], q3[2], q3[3]);
}
#endif

int sensor_update_time_ms = 6;

// TODO: get rid of it.. ?
static void set_update_time_ms(int time_ms)
{
	sensor_update_time_ms = time_ms; // TODO: terrible naming
}

int main_imu_init(void)
{
	int err;
	// TODO: on any errors set main_ok false and skip (make functions return nonzero)
	err = sensor_init(); // IMUs discovery
	if (err)
	{
		k_msleep(5);
		LOG_INF("Retrying sensor detection");

		// Reset address before retrying sensor detection
		sensor_imu_dev.addr = 0x00;

		err = sensor_init(); // on POR, the sensor may not be ready yet
		if (err)
			return err;
	}
	if (mag_available) // shutdown magnetometer first (in case of passthrough)
		sensor_mag->shutdown(); // TODO: is this needed?
	sensor_imu->shutdown(); // TODO: is this needed?

	float clock_actual_rate = 0;
#if CONFIG_USE_SENSOR_CLOCK
	set_sensor_clock(true, 32768, &clock_actual_rate); // enable the clock source for IMU if present
#endif
	if (clock_actual_rate != 0)
		LOG_INF("Sensor clock rate: %.2fHz", (double)clock_actual_rate);

	// wait for sensor register reset // TODO: is this needed?
	k_usleep(250);

	// set FS/range
	float accel_range = CONFIG_SENSOR_ACCEL_FS;
	float gyro_range = CONFIG_SENSOR_GYRO_FS;
	float accel_actual_range, gyro_actual_range;
	sensor_imu->update_fs(accel_range, gyro_range, &accel_actual_range, &gyro_actual_range);
	LOG_INF("Accelerometer range: %.2fg", (double)accel_actual_range);
	LOG_INF("Gyroscope range: %.2fdps", (double)gyro_actual_range);

	// setup sensor, set ODR
	float accel_initial_time = 1.0 / CONFIG_SENSOR_ACCEL_ODR; // configure with ~1000Hz ODR
	float gyro_initial_time = 1.0 / CONFIG_SENSOR_GYRO_ODR; // configure with ~1000Hz ODR
	float mag_initial_time = sensor_update_time_ms / 1000.0; // configure with ~200Hz ODR
	err = sensor_imu->init(clock_actual_rate, accel_initial_time, gyro_initial_time, &accel_actual_time, &gyro_actual_time);
	LOG_INF("Requested SPI frequency: %.2fMHz", (double)sensor_imu_spi_dev.config.frequency / 1000000.0);
	LOG_INF("Accelerometer initial rate: %.2fHz", 1.0 / (double)accel_actual_time);
	LOG_INF("Gyrometer initial rate: %.2fHz", 1.0 / (double)gyro_actual_time);
	if (err < 0)
		return err;
// 55-66ms to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
	if (mag_available && mag_enabled)
	{
		if (use_ext_fifo)
			sensor_imu->ext_passthrough(true); // reenable passthrough
		err = sensor_mag->init(mag_initial_time, &mag_actual_time); // configure with ~200Hz ODR
		LOG_INF("Magnetometer initial rate: %.2fHz", 1.0 / (double)mag_actual_time);
		if (err < 0)
			return err;
// 0-1ms to setup mmc
	}
	LOG_INF("Initialized sensors");

	// Setup fusion
	sensor_retained_read(); // TODO: useless
	if (fusion_id == FUSION_VQF)
		vqf_update_sensor_ids(sensor_imu_id);
	if (retained->fusion_id == fusion_id) // Check if the retained fusion data is valid and matches the selected fusion
	{ // Load state if the data is valid (fusion was initialized before)
		sensor_fusion->load(retained->fusion_data);
		retained->fusion_id = 0; // Invalidate retained fusion data
		retained_update();
	}
	else
	{
		sensor_fusion->init(gyro_actual_time, accel_actual_time, mag_initial_time); // TODO: using initial time since mag are not polled at the actual rate
	}

	sensor_calibration_update_sensor_ids(sensor_imu_id);
	if (sensor_imu == &sensor_imu_bmi270) // bmi270 specific
	{
		LOG_INF("Applying gyroscope gain");
		bmi_gain_apply(sensor_calibration_get_sensor_data());
	}

	LOG_INF("Using %s", fusion_names[fusion_id]);
	LOG_INF("Initialized fusion");
	sensor_fusion_init = true;
	return 0;
}

enum sensor_sensor_mode {
//	SENSOR_SENSOR_MODE_OFF,
	SENSOR_SENSOR_MODE_LOW_NOISE,
	SENSOR_SENSOR_MODE_LOW_POWER,
	SENSOR_SENSOR_MODE_LOW_POWER_2
};

static enum sensor_sensor_mode sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
static enum sensor_sensor_mode last_sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;

enum sensor_sensor_timeout {
	SENSOR_SENSOR_TIMEOUT_IMU,
	SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED,
};

static enum sensor_sensor_timeout sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;

static bool main_running = false;
static bool main_ok = false;
static bool send_info = false;

static int packet_errors = 0;

#define ACQUISITION_START_MS 1000
#define STATUS_INTERVAL_MS 5000

static int64_t last_status_time = 0;
static int64_t max_loop_time = 0;

#if DEBUG
static int64_t last_acquisition_time = INT64_MAX;
static uint64_t total_acquisition_time = 0;
static uint64_t total_read_packets = 0;
static uint64_t total_processed_packets = 0;
static uint64_t total_gyro_samples = 0;
static uint64_t total_accel_samples = 0;
#endif

void main_imu_thread(void)
{
	main_running = true;
	int err = main_imu_init(); // Initialize IMUs and Fusion
	// TODO: handle imu init error, maybe restart device?
	// TODO: on failure to init, disable sensor interface
	if (err)
		set_status(SYS_STATUS_SENSOR_ERROR, true); // TODO: only handles general init error
	else
		main_ok = true;
	while (1)
	{
		int64_t time_begin = k_uptime_get();
		if (main_ok)
		{
			// Resume devices
			sys_interface_resume();

			// Trigger reconfig on sensor mode change
			bool reconfig = last_sensor_mode != sensor_mode;
			last_sensor_mode = sensor_mode;

			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33 samples, low power)
			// Magneto sample will take ~400us
			// Fusing data will take between 100us (~7 samples, low noise) - 500us (~33 samples, low power) for xiofusion
			// TODO: on any errors set main_ok false and skip (make functions return nonzero)

			// At high speed, use oneshot mode to have synced magnetometer data
			// Call before FIFO and get the data after
			if (mag_available && mag_enabled && mag_use_oneshot)
				sensor_mag->mag_oneshot();

			// Read IMU temperature
			float temp = sensor_imu->temp_read(); // TODO: use as calibration data
			connection_update_sensor_temp(temp);

			// Read gyroscope (FIFO)
#if CONFIG_SENSOR_USE_LOW_POWER_2
			uint8_t* rawData = (uint8_t*)k_malloc(1900);  // Limit FIFO read to 2048 bytes (worst case is ICM 20 byte packet at 1000Hz and 100ms update time)
			if (rawData == NULL)
			{
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets = sensor_imu->fifo_read(rawData, 1900); // TODO: name this better?
#else
			uint8_t* rawData = (uint8_t*)k_malloc(1024);  // Limit FIFO read to 768 bytes (worst case is ICM 20 byte packet at 1000Hz and 33ms update time)
			if (rawData == NULL)
			{
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets = sensor_imu->fifo_read(rawData, 1024); // TODO: name this better?
#endif

			// Debug info
#if DEBUG
			int64_t acquisition_time = k_uptime_ticks();
			bool valid_acquisition = k_uptime_get() > ACQUISITION_START_MS && last_acquisition_time < acquisition_time; // wait before beginning profiling
			if (valid_acquisition)
			{
				total_acquisition_time += acquisition_time - last_acquisition_time;
				total_read_packets += packets;
			}
			last_acquisition_time = acquisition_time;
#endif

			// Read magnetometer
			float raw_m[3];
			if (mag_available && mag_enabled && sensor_mode == SENSOR_SENSOR_MODE_LOW_NOISE)
				sensor_mag->mag_read(raw_m); // reading mag last, and it will be processed last

			if (reconfig) // TODO: get rid of reconfig?
			{
				switch (sensor_mode)
				{
				case SENSOR_SENSOR_MODE_LOW_NOISE:
					set_update_time_ms(6);
					LOG_INF("Switching sensors to low noise");
					break;
				case SENSOR_SENSOR_MODE_LOW_POWER:
					set_update_time_ms(33);
					LOG_INF("Switching sensors to low power");
					if (mag_available && mag_enabled)
						sensor_mag->update_odr(INFINITY, &mag_actual_time); // standby/oneshot
					break;
				case SENSOR_SENSOR_MODE_LOW_POWER_2:
					set_update_time_ms(100);
					LOG_INF("Switching sensors to low power 2");
					if (mag_available && mag_enabled)
						sensor_mag->update_odr(INFINITY, &mag_actual_time); // standby/oneshot
					break;
				};
			}
			
			// Suspend devices
			sys_interface_suspend();

			// Fuse all data
			float a_sum[3] = {0};
			int a_count = 0;
			max_gyro_speed_square = 0;
			int processed_packets = 0;
			for (uint16_t i = 0; i < packets; i++) // TODO: fifo_process_ext is available, need to implement it
			{
				float raw_a[3] = {0};
				float raw_g[3] = {0};
				if (sensor_imu->fifo_process(i, rawData, raw_a, raw_g))
					continue; // skip on error

				// TODO: split into separate functions
				if (raw_g[0] != 0 || raw_g[1] != 0 || raw_g[2] != 0)
				{
#if DEBUG
					if (valid_acquisition)
						total_gyro_samples++;
#endif
					sensor_calibration_process_gyro(raw_g);
					float gx = raw_g[0];
					float gy = raw_g[1];
					float gz = raw_g[2];
					float g[] = {SENSOR_GYROSCOPE_AXES_ALIGNMENT};

					// Process fusion
					sensor_fusion->update_gyro(g, gyro_actual_time);

					if (mag_available && mag_enabled)
					{
						// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
						float g_off[3] = {};
						sensor_fusion->get_gyro_bias(g_off);
						for (int i = 0; i < 3; i++)
							g_off[i] = g[i] - g_off[i];
	
						// Get the highest gyro speed
						float gyro_speed_square = g_off[0] * g_off[0] + g_off[1] * g_off[1] + g_off[2] * g_off[2];
						if (gyro_speed_square > max_gyro_speed_square)
							max_gyro_speed_square = gyro_speed_square;
					}
				}

				if (raw_a[0] != 0 || raw_a[1] != 0 || raw_a[2] != 0)
				{
#if DEBUG
					if (valid_acquisition)
						total_accel_samples++;
#endif
					sensor_calibration_process_accel(raw_a);
					float ax = raw_a[0];
					float ay = raw_a[1];
					float az = raw_a[2];
					float a[] = {SENSOR_ACCELEROMETER_AXES_ALIGNMENT};

					// Process fusion
					sensor_fusion->update_accel(a, accel_actual_time);

					for (int i = 0; i < 3; i++)
						a_sum[i] += a[i];
					a_count++;
				}

				processed_packets++;
			}

			// Free the FIFO buffer
			k_free(rawData);

#if DEBUG
			if (valid_acquisition)
				total_processed_packets += processed_packets;
#endif

			if (mag_available && mag_enabled && sensor_mode == SENSOR_SENSOR_MODE_LOW_NOISE)
			{
				sensor_calibration_process_mag(raw_m);
				float mx = raw_m[0];
				float my = raw_m[1];
				float mz = raw_m[2];
				float m[] = {SENSOR_MAGNETOMETER_AXES_ALIGNMENT};

#ifndef SENSOR_SEND_RAW_MAG // XXX skip fusion for testing
				// Process fusion
				sensor_fusion->update_mag(m, sensor_update_time_ms / 1000.0); // TODO: use actual time?
#endif

				v_rotate(m, q3, m); // magnetic field in local device frame, no other transformation will be done
				connection_update_sensor_mag(m);
			}

			// Copy average acceleration for this frame
			static float a[3] = {0}; // keep persistent
			if (a_count > 0)
			{
				for (int i = 0; i < 3; i++)
					a[i] = a_sum[i] / a_count;
			}

			// Check packet processing
			if ((packets != 0 || k_uptime_get() > 100) && processed_packets == 0)
			{
				if (packets)
					LOG_WRN("No packets processed");
				else
					LOG_WRN("No packets in buffer");
				if (++packet_errors == 10)
				{
					LOG_ERR("Packet error threshold exceeded");
					set_status(SYS_STATUS_SENSOR_ERROR, true);
					if (packets)
					{
						sensor_retained_write(); // keep the fusion state
						sys_request_system_reboot();
					}
				}
			}
			else if (processed_packets == packets && packets > 0)
			{
				packet_errors = 0;
			}

			// Update fusion gyro sanity? // TODO: use to detect drift and correct or suspend tracking
//			sensor_fusion->update_gyro_sanity(g, m);

			// Get updated quaternion from fusion
			sensor_fusion->get_quat(q);
			q_normalize(q, q); // safe to use self as output

			// Get linear acceleration // TODO: move to util functions
			float lin_a[3] = {0};
			if (v_diff_mag(a, lin_a) != 0) // lin_a as zero vector
			{
				float vec_gravity[3] = {0};
				vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
				vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
				vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);
				for (int i = 0; i < 3; i++)
					lin_a[i] = (a[i] - vec_gravity[i]) * CONST_EARTH_GRAVITY; // vector to m/s^2
			}

			// Check the IMU gyroscope // TODO: gyro sanity not used
			bool calibrating = get_status(SYS_STATUS_CALIBRATION_RUNNING);
			bool resting = sensor_fusion->get_gyro_sanity() == 0 ? q_epsilon(q, last_q, 0.005) : q_epsilon(q, last_q, 0.05); // TODO: Probably okay to use the constantly updating last_q?
			if (!calibrating && resting)
			{
				int64_t last_data_delta = k_uptime_get() - last_data_time;
				if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER && last_data_delta > 500) // No motion in lp timeout
				{
					LOG_INF("No motion from sensors in %dms", CONFIG_SENSOR_LP_TIMEOUT);
					sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER;
				}
#if CONFIG_SENSOR_USE_LOW_POWER_2 || CONFIG_USE_IMU_TIMEOUT
				int64_t imu_timeout = CLAMP(last_data_time - last_suspend_attempt_time, CONFIG_IMU_TIMEOUT_RAMP_MIN, CONFIG_IMU_TIMEOUT_RAMP_MAX); // Ramp timeout from last_data_time
#endif
#if CONFIG_SENSOR_USE_LOW_POWER_2
				if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER_2 && last_data_delta > imu_timeout) // No motion in ramp time
					sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER_2;
#endif
#if CONFIG_USE_ACTIVE_TIMEOUT
				if (sensor_timeout < SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_ACTIVE_TIMEOUT_THRESHOLD) // higher priority than IMU timeout
				{
					LOG_INF("Switching to activity timeout");
					sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY;
				}
				if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_ACTIVE_TIMEOUT_DELAY)
				{
					LOG_INF("No motion from sensors in %dm", CONFIG_ACTIVE_TIMEOUT_DELAY / 60000);
#if CONFIG_SLEEP_ON_ACTIVE_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
					sys_request_WOM(true); // TODO: should queue shutdown and suspend itself instead
//					main_imu_suspend(); // TODO: auto suspend, the device should configure WOM ASAP but it does not
#elif CONFIG_SHUTDOWN_ON_ACTIVE_TIMEOUT && CONFIG_USER_SHUTDOWN
					main_running = false; // skip suspend step, at the moment the thread must be running to shutdown // TODO: should queue shutdown and suspend itself instead
					sys_request_system_off();
#endif
					sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED; // only try to suspend once
				}
#endif
#if CONFIG_USE_IMU_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
				if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU && last_data_delta > imu_timeout) // No motion in ramp time
				{
					LOG_INF("No motion from sensors in %llds", imu_timeout / 1000);
					sys_request_WOM(false); // TODO: should queue shutdown and suspend itself instead
//					main_imu_suspend(); // TODO: auto suspend, the device should configure WOM ASAP but it does not
					sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED; // only try to suspend once
				}
#endif
			}
			else
			{
				if (sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER_2 || sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED)
					last_suspend_attempt_time = k_uptime_get();
				last_data_time = k_uptime_get();
				if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED) // Resetting IMU timeout
					sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;
				sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
			}

			// Update magnetometer mode
			if (mag_available && mag_enabled && sensor_mode == SENSOR_SENSOR_MODE_LOW_NOISE)
			{
				float gyro_speed = sqrtf(max_gyro_speed_square);
				float mag_target_time = 1.0f / (4 * gyro_speed); // target mag ODR for ~0.25 deg error
				if (mag_target_time < 0.005f && mag_skip_oneshot) // only use continuous modes if oneshot is not available
					mag_target_time = 0.005;
				if (mag_target_time > 0.1f) // limit to 0.1 (minimum 10Hz)
					mag_target_time = 0.1;
				sys_interface_resume();
				if (mag_target_time < 0.005f) // cap at 0.005 (200Hz), above this the sensor will use oneshot mode instead
				{
					int err = sensor_mag->update_odr(INFINITY, &mag_actual_time);
					if (mag_actual_time == INFINITY)
					{
						if (!err)
							LOG_DBG("Switching magnetometer to oneshot");
						mag_use_oneshot = true;
					}
					else // magnetometer did not have a oneshot mode, try 200Hz
					{
						if (!err)
							mag_skip_oneshot = true;
						mag_target_time = 0.005;
					}
				}
				if (mag_target_time >= 0.005f || mag_actual_time != INFINITY) // under 200Hz or magnetometer did not have a oneshot mode
				{
					int err = sensor_mag->update_odr(mag_target_time, &mag_actual_time);
					if (!err)
						LOG_DBG("Switching magnetometer ODR to %.2fHz", 1.0 / (double)mag_actual_time);
					mag_use_oneshot = false;
				}
				sys_interface_suspend();
			}

			// Check if last status is outdated
			if (!send_info && (k_uptime_get() - last_info_time > 100))
			{
				send_info = true;
				last_info_time = k_uptime_get();
			}

			// Send packet with new orientation
			bool send_quat_data = !q_epsilon(q, last_q, 0.001);
			bool send_lin_accel_data = !v_epsilon(lin_a, last_lin_a, 0.05);
			if (send_quat_data || send_lin_accel_data)
			{
				bool send_precise_quat = q_epsilon(q, last_q, 0.005);
				memcpy(last_q, q, sizeof(q));
				memcpy(last_lin_a, lin_a, sizeof(lin_a));
				float q_offset[4];
				q_multiply(q, q3, q_offset); // quaternion in device orientation, connection will change format from wxyz to xyzw
				v_rotate(lin_a, q3, lin_a); // linear acceleration in local device frame, no other transformation will be done
				connection_update_sensor_data(q_offset, lin_a);
				if (send_info && !send_precise_quat) // prioritize quat precision
				{
					connection_write_packet_2();
					send_info = false;
				}
				else if (mag_available && mag_enabled && k_uptime_get() - last_mag_time > 200) // try to send mag data every 200ms
				{
					connection_write_packet_4();
					last_mag_time = k_uptime_get();
				}
				else
				{
					connection_write_packet_1();
				}
			}
			else if (send_info)
			{
				connection_write_packet_0();
				send_info = false;
			}
			else
			{
				connection_clocks_request_stop();
			}

			// Handle magnetometer calibration
			if (mag_available && mag_enabled && last_sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER && sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER)
				sensor_request_calibration_mag();
		}

		main_running = false;
		int64_t time_delta = k_uptime_get() - time_begin;

		if (time_delta > sensor_update_time_ms)
			max_loop_time = MAX(max_loop_time, time_delta);

		if (k_uptime_get() - last_status_time > STATUS_INTERVAL_MS)
		{
			last_status_time = k_uptime_get();
			if (max_loop_time > 0)
			{
				LOG_WRN("Last update steps took up to %lld ms", time_delta);
				max_loop_time = 0;
			}
#if DEBUG
			LOG_DBG("packets read: %llu, processed: %llu, gyro samples: %llu, accel samples: %llu, total acquisition time: %lld us", total_read_packets, total_processed_packets, total_gyro_samples, total_accel_samples, k_ticks_to_us_near64(total_acquisition_time));
			LOG_DBG("reported gyro rate: %.2fHz, actual: %.2fHz, reported accel rate: %.2fHz, actual: %.2fHz", 1.0 / (double)gyro_actual_time, (double)total_gyro_samples / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0, 1.0 / (double)accel_actual_time, (double)total_accel_samples / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0);
#endif
		}

//		led_clock_offset += time_delta;
		if (time_delta > sensor_update_time_ms)
			k_yield();
		else
			k_msleep(sensor_update_time_ms - time_delta);

		if (main_suspended) // TODO:
			k_thread_suspend(main_imu_thread_id);

		main_running = true;
	}
}

void wait_for_threads(void) // TODO: add timeout
{
	while (main_running)
		k_usleep(1); // bane of my existence. don't use k_yield()!!!!!!
}

void main_imu_suspend(void) // TODO: add timeout
{
	main_suspended = true;
	if (!main_running) // don't suspend if already stopped (TODO: may be called from sensor thread)
		return;
	while (sensor_sensor_scanning)
		k_usleep(1); // try not to interrupt scanning
	while (main_running) // TODO: change to detect if i2c is busy
		k_usleep(1); // try not to interrupt anything actually
	k_thread_suspend(main_imu_thread_id);
	LOG_INF("Suspended sensor thread");
}

void main_imu_resume(void)
{
	if (!main_suspended) // not suspended
		return;
	k_thread_resume(main_imu_thread_id);
	LOG_INF("Resumed sensor thread");
}

void main_imu_wakeup(void)
{
	if (!main_suspended) // don't wake up if pending suspension
		k_wakeup(main_imu_thread_id);
}
