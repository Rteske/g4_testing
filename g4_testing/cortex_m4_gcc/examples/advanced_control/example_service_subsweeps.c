// Copyright (c) Acconeer AB, 2021-2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_config.h"
#include "acc_config_subsweep.h"
#include "acc_definitions_a121.h"
#include "acc_definitions_common.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_processing.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"

#include "acc_version.h"


/** \example example_service_subsweeps.c
 * @brief This is an example on how to use subsweeps with the Service API
 * @n
 * The example executes as follows:
 *   - Create a configuration
 *   - Configure multiple subsweeps
 *   - Create a processing instance using the previously created configuration
 *   - Create a sensor instance
 *   - Prepare a sensor
 *   - Perform a sensor measurement and read out the data
 *   - Process the measurement subsweeps
 *   - Destroy the sensor instance
 *   - Destroy the processing instance
 *   - Destroy the configuration
 */

#define SENSOR_ID          (1U)
#define SENSOR_TIMEOUT_MS  (1000U)
#define MAX_DATA_ENTRY_LEN 15 // "-32000+-32000i" + zero termination

static void print_data(acc_int16_complex_t *data, uint16_t data_length);


static void cleanup(acc_config_t *config, acc_processing_t *processing,
                    acc_sensor_t *sensor, void *buffer);


int acc_example_service_subsweeps(int argc, char *argv[]);


int acc_example_service_subsweeps(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	acc_config_t              *config     = NULL;
	acc_processing_t          *processing = NULL;
	acc_sensor_t              *sensor     = NULL;
	void                      *buffer     = NULL;
	uint32_t                  buffer_size = 0;
	acc_processing_metadata_t proc_meta;
	acc_processing_result_t   proc_result;

	printf("Acconeer software version %s\n", acc_version_get());

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	config = acc_config_create();
	if (config == NULL)
	{
		printf("acc_config_create() failed\n");
		cleanup(config, processing, sensor, buffer);
		return EXIT_FAILURE;
	}

	// Collect 16 sweeps per frame, applicable for all subsweeps
	acc_config_sweeps_per_frame_set(config, 16);

	// Setup 3 subsweeps with different gains, start points and number of points
	acc_config_num_subsweeps_set(config, 3);

	acc_config_subsweep_receiver_gain_set(config, 5, 0);
	acc_config_subsweep_start_point_set(config, 100, 0);
	acc_config_subsweep_num_points_set(config, 5, 0);

	acc_config_subsweep_receiver_gain_set(config, 15, 1);
	acc_config_subsweep_start_point_set(config, 300, 1);
	acc_config_subsweep_num_points_set(config, 20, 1);

	acc_config_subsweep_receiver_gain_set(config, 23, 2);
	acc_config_subsweep_start_point_set(config, 500, 2);
	acc_config_subsweep_num_points_set(config, 10, 2);

	// Print the configuration
	acc_config_log(config);

	processing = acc_processing_create(config, &proc_meta);
	if (processing == NULL)
	{
		printf("acc_processing_create() failed\n");
		cleanup(config, processing, sensor, buffer);
		return EXIT_FAILURE;
	}

	if (!acc_rss_get_buffer_size(config, &buffer_size))
	{
		printf("acc_rss_get_buffer_size() failed\n");
		cleanup(config, processing, sensor, buffer);
		return EXIT_FAILURE;
	}

	buffer = acc_integration_mem_alloc(buffer_size);
	if (buffer == NULL)
	{
		printf("buffer allocation failed\n");
		cleanup(config, processing, sensor, buffer);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	sensor = acc_sensor_create(SENSOR_ID);
	if (sensor == NULL)
	{
		printf("acc_sensor_create() failed\n");
		cleanup(config, processing, sensor, buffer);
		return EXIT_FAILURE;
	}

	bool             status;
	bool             cal_complete = false;
	acc_cal_result_t cal_result;

	do
	{
		status = acc_sensor_calibrate(sensor, &cal_complete, &cal_result, buffer, buffer_size);

		if (status && !cal_complete)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !cal_complete);

	if (!status)
	{
		printf("acc_sensor_calibrate() failed\n");
		acc_sensor_status(sensor);
		cleanup(config, processing, sensor, buffer);
		return EXIT_FAILURE;
	}

	// Reset sensor after calibration by disabling it
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	if (!acc_sensor_prepare(sensor, config, &cal_result, buffer, buffer_size))
	{
		printf("acc_sensor_prepare() failed\n");
		acc_sensor_status(sensor);
		cleanup(config, processing, sensor, buffer);
		return EXIT_FAILURE;
	}

	for (uint32_t i = 0U; i < 5U; i++)
	{
		if (!acc_sensor_measure(sensor))
		{
			printf("acc_sensor_measure failed\n");
			acc_sensor_status(sensor);
			cleanup(config, processing, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			acc_sensor_status(sensor);
			cleanup(config, processing, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_read(sensor, buffer, buffer_size))
		{
			printf("acc_sensor_read failed\n");
			acc_sensor_status(sensor);
			cleanup(config, processing, sensor, buffer);
			return EXIT_FAILURE;
		}

		acc_processing_execute(processing, buffer, &proc_result);

		// The number of sweeps can also be readout with acc_config_sweeps_per_frame_get()
		uint16_t sweeps_per_frame    = proc_meta.frame_data_length / proc_meta.sweep_data_length;
		uint8_t  number_of_subsweeps = acc_config_num_subsweeps_get(config);

		for (uint16_t sweep_index = 0; sweep_index < sweeps_per_frame; sweep_index++)
		{
			for (uint8_t subsweep_index = 0; subsweep_index < number_of_subsweeps; subsweep_index++)
			{
				/* Each sweep is offsetted sweep_data_length into the frame,
				   and within the sweep the specific subsweep is offsetted
				   subsweep_data_offset. */
				uint16_t subsweep_offset = sweep_index * proc_meta.sweep_data_length +
				                           proc_meta.subsweep_data_offset[subsweep_index];
				uint16_t            subsweep_length = proc_meta.subsweep_data_length[subsweep_index];
				acc_int16_complex_t *subsweep_data  = &proc_result.frame[subsweep_offset];

				printf("Subsweep: %" PRIu8 ", sweep: %" PRIu16 ", processed data:\n", subsweep_index, sweep_index);
				print_data(subsweep_data, subsweep_length);
			}
		}
	}

	cleanup(config, processing, sensor, buffer);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}


static void cleanup(acc_config_t *config, acc_processing_t *processing,
                    acc_sensor_t *sensor, void *buffer)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	if (sensor != NULL)
	{
		acc_sensor_destroy(sensor);
	}

	if (processing != NULL)
	{
		acc_processing_destroy(processing);
	}

	if (config != NULL)
	{
		acc_config_destroy(config);
	}

	if (buffer != NULL)
	{
		acc_integration_mem_free(buffer);
	}
}


static void print_data(acc_int16_complex_t *data, uint16_t data_length)
{
	char buffer[MAX_DATA_ENTRY_LEN];

	for (uint16_t i = 0; i < data_length; i++)
	{
		if ((i > 0) && ((i % 8) == 0))
		{
			printf("\n");
		}

		snprintf(buffer, sizeof(buffer), "%" PRIi16 "+%" PRIi16 "i", data[i].real, data[i].imag);

		printf("%14s ", buffer);
	}

	printf("\n");
}
