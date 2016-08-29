
// for memset
#include <string.h>

// for malloc/free
#include <stdlib.h>

// for DBL_MAX
#include <float.h>

#include "vlp16Ctrl.h"

static void clear_communication_status_of_vlp16_handler(vlp16_handler_t *handler)
{
    handler->communication_status.buffer_error_occurs = false;
    handler->communication_status.decode_error_occurs = false;
    handler->communication_status.memory_allocated = false;
    handler->communication_status.socket_opened = false;

    return;
}

static void clear_vlp16_decode_buffer(vlp16_handler_t *handler)
{
    memset((void *)handler->decode_buffer, 0,
           VLP16_PACKET_LENGTH);

    for (unsigned int i = 0; i < VLP16_PACKET_NUMBER_OF_DATA_BLOCKS; ++i) {
        handler->decoding_data_blocks[i] = NULL;
    }

    return;
}

static void clear_vlp16_remaining_data_blocks(vlp16_handler_t *handler)
{
    handler->number_of_remaining_data_blocks = 0;

    for (unsigned int i = 0; i < NUMBER_OF_STORING_VLP16_PACKET_DATA_BLOCKS; ++i) {
        memset((void *)handler->remaining_data_block_buffer[i], 0,
               VLP16_PACKET_DATA_BLOCK_LENGTH);
    }

    return;
}

void clear_vlp16_handler(vlp16_handler_t *handler)
{
    clear_communication_status_of_vlp16_handler(handler);

    clear_vlp16_remaining_data_blocks(handler);
    clear_vlp16_decode_buffer(handler);

    handler->decoding_packet_timestamp_usec = 0;
    handler->past_packet_timestamp_usec = 0;

    handler->decoding_packet_return_mode = VLP16_PACKET_INVALID_RETURN_MODE;
    handler->decoding_packet_sensor_model = VLP16_PACKET_INVALID_SENSOR_MODEL;

    handler->past_packet_return_mode = VLP16_PACKET_INVALID_RETURN_MODE;
    handler->past_packet_sensor_model = VLP16_PACKET_INVALID_SENSOR_MODEL;

    handler->minimum_elevation_angle = 0.0;
    handler->maximum_elevation_angle = 0.0;
    if (handler->elevation_angle_array.size() != 0) {
        handler->elevation_angle_array.clear();
        handler->elevation_angle_array.reserve(VLP16_PACKET_NUMBER_OF_SPOTS[VLP16_PACKET_HDL_32E]);
    }

    handler->timer.SetIntervalStart();
    handler->no_reply_interval_timer.SetIntervalStart();

    initialize_lidar_line_circular_buffer(&handler->line_data_buffer);

    return;
}

static bool allocate_memory_for_lidar_line_circular_buffer(vlp16_handler_t *vlp16_handler,
                                                           unsigned int number_of_spots,
                                                           unsigned int number_of_lines)
{
    if (is_allocated_memory_of_lidar_line_circular_buffer(&vlp16_handler->line_data_buffer) == true) {

        if (number_of_spots == VLP16_PACKET_NUMBER_OF_SPOTS[vlp16_handler->decoding_packet_sensor_model]) {
            return true;
        }

        release_memory_of_lidar_line_circular_buffer(&vlp16_handler->line_data_buffer);
    }

    return allocate_memory_for_lidar_line_circular_buffer(&vlp16_handler->line_data_buffer,
                                                          number_of_spots, number_of_lines);
}

bool allocate_circular_buffer_for_vlp16_handler(vlp16_handler_t *vlp16_handler, enum VLP16_PACKET_SENSOR_MODEL sensor_model,
                                                unsigned int buffer_length_byte)
{
    if (buffer_length_byte < VLP16_PACKET_LENGTH) {
        vlp16_handler->communication_status.memory_allocated = false;
        vlp16_handler->communication_status.buffer_error_occurs = true;
        return false;
    }

    if ((sensor_model == VLP16_PACKET_INVALID_SENSOR_MODEL) ||
        (sensor_model == NUMBER_OF_SENSOR_MODELS_IN_VLP16_PACKET)) {
        return false;
    }

    bool return_bool =
        allocate_circular_receive_buffer_of_socket_client(&vlp16_handler->socket_handler,
                                                          buffer_length_byte);
    if (return_bool == false) {
        vlp16_handler->communication_status.memory_allocated = false;
        return false;
    }

    return_bool =
        allocate_memory_for_lidar_line_circular_buffer(vlp16_handler,
                                                       VLP16_PACKET_NUMBER_OF_SPOTS[sensor_model],
                                                       NUMBER_OF_VLP16_PACKET_LINES_TO_STORE_MEASURED_DATA);

    if (return_bool == false) {
        release_circular_buffer_of_vlp16_handler(vlp16_handler);
        return false;
    }

    vlp16_handler->communication_status.memory_allocated = true;

    return true;
}

bool release_circular_buffer_of_vlp16_handler(vlp16_handler_t *vlp16_handler)
{

    bool return_bool = false;
    if (vlp16_handler->communication_status.memory_allocated == true) {

        return_bool =
            release_memory_of_circular_buffer(&vlp16_handler->socket_handler.buffer);

        vlp16_handler->communication_status.memory_allocated = false;

        if (is_allocated_memory_of_lidar_line_circular_buffer(&vlp16_handler->line_data_buffer) == true) {
            release_memory_of_lidar_line_circular_buffer(&vlp16_handler->line_data_buffer);
        }

        return return_bool;
    }

    return return_bool;
}

bool open_socket_for_vlp16_handler(const char *destination_ip_address_string,
                                   const char *destination_port_number_string,
                                   const char *reception_ip_address_string,
                                   const char *reception_port_number_string,
                                   int communication_timeout_usec,
                                   vlp16_handler_t *vlp16_handler)
{
    if (vlp16_handler->communication_status.socket_opened == true) {
        return true;
    }

    clear_vlp16_decode_buffer(vlp16_handler);
    clear_vlp16_remaining_data_blocks(vlp16_handler);

    if (open_socket_for_client(destination_ip_address_string, destination_port_number_string,
                               reception_ip_address_string, reception_port_number_string,
                               SOCKET_PROTOCOL_UDP,
                               &vlp16_handler->socket_handler) == false) {

        vlp16_handler->communication_status.socket_opened = false;

        return false;

    }

    vlp16_handler->communication_timeout_usec = communication_timeout_usec;
    vlp16_handler->communication_status.socket_opened = true;

    return true;
}

void close_socket_of_vlp16_handler(vlp16_handler_t *vlp16_handler)
{

    close_socket_of_client(&vlp16_handler->socket_handler);

    vlp16_handler->communication_status.socket_opened = false;

    return;
}

void erase_all_receiving_and_decoding_data(vlp16_handler_t *handler)
{
    clear_vlp16_decode_buffer(handler);
    clear_vlp16_remaining_data_blocks(handler);
    clear_memory_of_circular_buffer(&handler->socket_handler.buffer);

    return;
}

static enum VLP16_PACKET_RETURN_MODE get_return_mode_in_vlp16_packet(const char*data)
{
    enum VLP16_PACKET_RETURN_MODE return_mode = VLP16_PACKET_INVALID_RETURN_MODE;

    for (unsigned int i = 0; i < NUMBER_OF_RETURN_MODES_IN_VLP16_PACKET; ++i) {

        if (*data == VLP16_PACKET_RETURN_MODE_BYTE[i]) {
            return_mode = (enum VLP16_PACKET_RETURN_MODE) i;
            break;
        }

    }

    return return_mode;
}

static enum VLP16_PACKET_SENSOR_MODEL get_sensor_model_in_vlp16_packet(const char*data)
{
    enum VLP16_PACKET_SENSOR_MODEL sensor_model = VLP16_PACKET_INVALID_SENSOR_MODEL;

    for (unsigned int i = 0; i < NUMBER_OF_SENSOR_MODELS_IN_VLP16_PACKET; ++i) {

        if (*data == VLP16_PACKET_SENSOR_MODEL_BYTE[i]) {
            sensor_model = (enum VLP16_PACKET_SENSOR_MODEL) i;
            break;
        }

    }

    return sensor_model;
}

static void decode_timestamp_and_return_mode_and_sensor_model_of_vlp16_packet(vlp16_handler_t *handler)
{
    const char *packet = handler->decode_buffer;

    handler->decoding_packet_timestamp_usec =
        decode_unsigned_value(packet + VLP16_PACKET_TIMESTAMP_POSITION,
                              VLP16_PACKET_TIMESTAMP_LENGTH, false);

    handler->decoding_packet_return_mode =
        get_return_mode_in_vlp16_packet(packet + VLP16_PACKET_RETURN_MODE_BYTE_POSITION);

    handler->decoding_packet_sensor_model =
        get_sensor_model_in_vlp16_packet(packet + VLP16_PACKET_SENSOR_MODEL_BYTE_POSITION);

    return;
}

static bool verify_flags_of_data_blocks(const vlp16_handler_t *vlp16_handler)
{
    const char *packet_data = vlp16_handler->decode_buffer;

    for (unsigned int block_index = 0; block_index < VLP16_PACKET_NUMBER_OF_DATA_BLOCKS; ++block_index) {

        const char *block_start =
            packet_data + VLP16_PACKET_DATA_BLOCK_POSITION[block_index];

        if (are_equivalent_byte_sequences(block_start, VLP16_PACKET_HEADER_FLAG_OF_DATA_BLOCK,
                                          VLP16_PACKET_HEADER_FLAG_OF_DATA_BLOCK_LENGTH) == false) {
            return false;
        }
    }

    return true;
}

bool receive_vlp16_packet(vlp16_handler_t *vlp16_handler, int *received_data_length,
                          unsigned int onetime_receive_length_byte)
{
    const unsigned awaiting_message_length = VLP16_PACKET_LENGTH;
    char *copy_destination = vlp16_handler->decode_buffer;

    unsigned int captured_data_length = 0;
    unsigned int copied_data_length = 0;

    // receive constant length data
    *received_data_length =
        receive_constant_length_data(&vlp16_handler->socket_handler, vlp16_handler->communication_timeout_usec,
                                     awaiting_message_length, copy_destination,
                                     onetime_receive_length_byte,
                                     &captured_data_length, &copied_data_length);

    if (copied_data_length != awaiting_message_length) {
        vlp16_handler->communication_status.decode_error_occurs = true;
        return false;
    }

    if (verify_flags_of_data_blocks(vlp16_handler) == false) {
        vlp16_handler->communication_status.decode_error_occurs = true;
        return false;
    }

    // reset no reply interval timer
    vlp16_handler->no_reply_interval_timer.SetIntervalStart();

    // renew packet information
    decode_timestamp_and_return_mode_and_sensor_model_of_vlp16_packet(vlp16_handler);


    // renew data block accessor
    const char *packet_data = vlp16_handler->decode_buffer;

    for (unsigned int block_index = 0; block_index < VLP16_PACKET_NUMBER_OF_DATA_BLOCKS; ++block_index) {

        vlp16_handler->decoding_data_blocks[block_index] =
            packet_data + VLP16_PACKET_DATA_BLOCK_POSITION[block_index];

    }

    return true;
}

static unsigned int calculate_azimuthal_angle_difference(unsigned int start_angle, unsigned int end_angle)
{
    unsigned int calculation_end_angle = end_angle;
    if (start_angle > end_angle) {
        calculation_end_angle = end_angle + VLP16_PACKET_MAXIMUM_AZIMUTHAL_ANGLE;
    }

    return calculation_end_angle - start_angle;
}

static void decode_azimuthal_angles_of_single_echo_vlp16_packet(const char **concatenated_data_blocks, unsigned int length_of_concatenated_data_blocks,
                                                                unsigned int *angle_buffer)
{
    const char *azimuthal_angle_start = NULL;
    for (unsigned int i = 0; i < length_of_concatenated_data_blocks; ++i) {

        azimuthal_angle_start =
            concatenated_data_blocks[i] + VLP16_PACKET_AZIMUTHAL_ANGLE_POSITION_IN_DATA_BLOCK;

        angle_buffer[i] =
            decode_unsigned_value(azimuthal_angle_start,
                                  VLP16_PACKET_AZIMUTHAL_ANGLE_LENGTH, false);
    }

    return;
}

static void make_default_vlp16_elevation_angle_array(const VLP16_PACKET_SENSOR_MODEL sensor_model,
                                                     std::vector<double> &angle_array, double *minimum_angle, double *maximum_angle)
{
    switch (sensor_model) {

        case VLP16_PACKET_HDL_32E:
        case VLP16_PACKET_VLP16:
            if (angle_array.size() != VLP16_PACKET_NUMBER_OF_SPOTS[sensor_model]) {
                angle_array.resize(VLP16_PACKET_NUMBER_OF_SPOTS[sensor_model]);
            }

            break;

        default:
            angle_array.clear();
            return;
    }

    if (sensor_model == VLP16_PACKET_VLP16) {
        *minimum_angle = DBL_MAX;
        *maximum_angle = -DBL_MAX;

        for (unsigned int spot_index = 0; spot_index < VLP16_PACKET_NUMBER_OF_SPOTS[sensor_model]; ++spot_index) {
            angle_array.at(spot_index) =
                VLP16_SPOT_SPECIFIVATION_ELEVATION_ANGLE_DEGREE_ARRAY[spot_index] * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

            if (angle_array.at(spot_index) < *minimum_angle) {
                *minimum_angle = angle_array.at(spot_index);
            }
            if (angle_array.at(spot_index) > *maximum_angle) {
                *maximum_angle = angle_array.at(spot_index);
            }

        }

    } else if (sensor_model == VLP16_PACKET_HDL_32E) {

    }

    return;
}

static void decode_one_line_data_of_single_echo_vlp16_packet(vlp16_handler_t *vlp16_handler,
                                                             unsigned int line_start_timestamp, double start_azimuthal_angle,
                                                             double one_spot_azimuthal_angle_step,
                                                             const char *data_buffer)
{

    lidar_line_circular_buffer_t *line_data_buffer = &vlp16_handler->line_data_buffer;
    lidar_line_data_t *line_data =
        get_pointer_to_copy_lidar_line_data(line_data_buffer,
                                            LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE);

    line_data->minimum_horizontal_angle = start_azimuthal_angle;

    line_data->next_data_index = 0;

    unsigned int spot_index = 0;

    unsigned int echo_buffer_index = line_data->next_data_index;
    lidar_echo_data_t *echo_buffer = line_data->echo_buffer;

    lidar_echo_data_t *echo = &echo_buffer[echo_buffer_index];
    unsigned int echo_index = 0;

    lidar_spot_accessor_t *spot = &line_data->spot[spot_index];

    unsigned int binary_data_index = 0;

    const unsigned int number_of_spots = VLP16_PACKET_NUMBER_OF_SPOTS[vlp16_handler->decoding_packet_sensor_model];
    line_data->maximum_horizontal_angle =
        start_azimuthal_angle + one_spot_azimuthal_angle_step * (double)(number_of_spots - 1);

    if (vlp16_handler->elevation_angle_array.size() != number_of_spots) {
        make_default_vlp16_elevation_angle_array(vlp16_handler->decoding_packet_sensor_model,
                                                 vlp16_handler->elevation_angle_array,
                                                 &vlp16_handler->minimum_elevation_angle,
                                                 &vlp16_handler->maximum_elevation_angle);
    }
    line_data->minimum_elevation_angle = vlp16_handler->minimum_elevation_angle;
    line_data->maximum_elevation_angle = vlp16_handler->maximum_elevation_angle;

    // todo set elevation angle
    for (unsigned int spot_index = 0; spot_index < number_of_spots; ++spot_index) {
        spot = &line_data->spot[spot_index];

        spot->number_of_echoes = 1;

        echo_index = 0;
        echo = &echo_buffer[echo_buffer_index];

        echo->index = echo_index + 1;
        echo->number_of_echoes_at_same_time = spot->number_of_echoes;

        echo->measured_time = VLP16_PACKET_ONE_LASER_FIRING_INTERVAL_USEC * (double)spot_index + (double)line_start_timestamp;
        echo->calibrated_time = echo->measured_time;

        echo->horizontal_angle = one_spot_azimuthal_angle_step * (double)(spot_index) + start_azimuthal_angle;

        echo->elevation_angle = vlp16_handler->elevation_angle_array.at(spot_index);

        echo->distance = VLP16_PACKET_DISTANCE_SCALE *
            (double)decode_unsigned_value(data_buffer + binary_data_index,
                                          VLP16_PACKET_DISTANCE_LENGTH, false);
        binary_data_index += VLP16_PACKET_DISTANCE_LENGTH;

        echo->intensity = (double)decode_unsigned_value(data_buffer + binary_data_index,
                                                        VLP16_PACKET_REFLECTIVITY_LENGTH, false);
        binary_data_index += VLP16_PACKET_REFLECTIVITY_LENGTH;

        spot->echo[echo_index] = (int)echo_buffer_index;
        ++echo_buffer_index;
        ++echo_index;

        for (; echo_index < LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES; ++echo_index) {
            spot->echo[echo_index] = LIDAR_SPOT_ACCESSOR_INVALID_ECHO_INDEX;
        }

    }
    move_copy_destination_point(line_data_buffer, 1);

    return;
}

static unsigned int decode_one_data_block_of_single_echo_vlp16_packet(vlp16_handler_t *vlp16_handler,
                                                                      unsigned int data_block_start_timestamp,
                                                                      unsigned int start_azimuthal_angle, unsigned int end_azimuthal_angle,
                                                                      const char *data_buffer)
{
    unsigned int captured_line_count = 0;

    const unsigned int azimuthal_angle_difference =
        calculate_azimuthal_angle_difference(start_azimuthal_angle, end_azimuthal_angle);

    const double one_spot_azimuthal_angle_step =
        VLP16_PACKET_AZIMUTHAL_ANGLE_SCALE * VLP16_PACKET_ONE_LASER_FIRING_RATIO_TO_ONE_FIRING_SEQUENCE * (double)azimuthal_angle_difference;

    const double scaled_start_azimuthal_angle =
        VLP16_PACKET_AZIMUTHAL_ANGLE_SCALE * (double)start_azimuthal_angle;

    // decode first line
    decode_one_line_data_of_single_echo_vlp16_packet(vlp16_handler,
                                                     data_block_start_timestamp, scaled_start_azimuthal_angle,
                                                     one_spot_azimuthal_angle_step,
                                                     data_buffer + VLP16_PACKET_ODD_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK);
    ++captured_line_count;

    // decode second line
    if (vlp16_handler->decoding_packet_sensor_model == VLP16_PACKET_VLP16) {

        const double start_line_azimuthal_angle_start =
            VLP16_PACKET_AZIMUTHAL_ANGLE_SCALE * (double)(start_azimuthal_angle + (azimuthal_angle_difference / 2));

        decode_one_line_data_of_single_echo_vlp16_packet(vlp16_handler,
                                                         data_block_start_timestamp + (unsigned int)VLP16_PACKET_LASER_FIRING_SEQUENCE_USEC,
                                                         start_line_azimuthal_angle_start,
                                                         one_spot_azimuthal_angle_step,
                                                         data_buffer + VLP16_PACKET_EVEN_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK);
        ++captured_line_count;
    }

    return captured_line_count;
}

static unsigned int decode_single_echo_vlp16_packet(vlp16_handler_t *vlp16_handler)
{
    const unsigned int length_of_concatenated_data_blocks =
        VLP16_PACKET_NUMBER_OF_DATA_BLOCKS + vlp16_handler->number_of_remaining_data_blocks;
    const char ** concatenated_data_blocks =
        (const char**)malloc(length_of_concatenated_data_blocks * sizeof(const char *));

    if (concatenated_data_blocks == NULL) {
        return 0;
    }

    for (unsigned int i = 0; i < vlp16_handler->number_of_remaining_data_blocks; ++i) {
        concatenated_data_blocks[i] = vlp16_handler->remaining_data_block_buffer[i];
    }

    for (unsigned int i = 0; i < VLP16_PACKET_NUMBER_OF_DATA_BLOCKS; ++i) {
        concatenated_data_blocks[i + vlp16_handler->number_of_remaining_data_blocks] = vlp16_handler->decoding_data_blocks[i];
    }

    unsigned int *angle_buffer = NULL;
    angle_buffer =
        (unsigned int *) malloc(length_of_concatenated_data_blocks * sizeof(unsigned int));

    decode_azimuthal_angles_of_single_echo_vlp16_packet(concatenated_data_blocks, length_of_concatenated_data_blocks,
                                                        angle_buffer);

    unsigned int angle_difference = 0;
    unsigned int start_offset = 0;
    for (start_offset = 0; start_offset < vlp16_handler->number_of_remaining_data_blocks; ++start_offset) {

        angle_difference =
            calculate_azimuthal_angle_difference(angle_buffer[vlp16_handler->number_of_remaining_data_blocks - start_offset - 1],
                                                 angle_buffer[vlp16_handler->number_of_remaining_data_blocks - start_offset]);

        if (angle_difference > VLP16_PACKET_AZIMUTHAL_ANGLE_THRESHOLD_OF_CONTINUOUS_DATA_BLOCKS) {
            break;
        }
    }
    const unsigned int start_data_index = vlp16_handler->number_of_remaining_data_blocks - start_offset;

    unsigned int end_offset = 0;
    for (end_offset = 0; end_offset < VLP16_PACKET_NUMBER_OF_DATA_BLOCKS - 1; ++end_offset) {

        angle_difference =
            calculate_azimuthal_angle_difference(angle_buffer[vlp16_handler->number_of_remaining_data_blocks + end_offset],
                                                 angle_buffer[vlp16_handler->number_of_remaining_data_blocks + end_offset + 1]);

        if (angle_difference > VLP16_PACKET_AZIMUTHAL_ANGLE_THRESHOLD_OF_CONTINUOUS_DATA_BLOCKS) {
            break;
        }
    }
    const unsigned int end_data_index_out = vlp16_handler->number_of_remaining_data_blocks + end_offset;

    unsigned int start_data_block_timestamp_usec = vlp16_handler->decoding_packet_timestamp_usec;
    if (start_offset > 0) {

        unsigned int offset_of_timestamp = (unsigned int)(VLP16_PACKET_ONE_DATA_BLOCK_USEC * (double)start_offset);

        if (start_data_block_timestamp_usec < offset_of_timestamp) {
            start_data_block_timestamp_usec = VLP16_PACKET_MAXIMUM_TIMESTAMP + start_data_block_timestamp_usec - offset_of_timestamp;
        } else {
            start_data_block_timestamp_usec = start_data_block_timestamp_usec - offset_of_timestamp;
        }

    }

    unsigned int data_block_start_timestamp = start_data_block_timestamp_usec;

    unsigned int number_of_captured_lines = 0;
    for (unsigned int i = start_data_index; i < end_data_index_out; ++i) {

        data_block_start_timestamp = start_data_block_timestamp_usec +
            (unsigned int)(VLP16_PACKET_ONE_DATA_BLOCK_USEC * (double)i);

        number_of_captured_lines += decode_one_data_block_of_single_echo_vlp16_packet(vlp16_handler,
                                                                                      data_block_start_timestamp,
                                                                                      angle_buffer[i], angle_buffer[i + 1],
                                                                                      concatenated_data_blocks[i]);

    }

    const unsigned int temporal_number_of_remaining_data_blocks =
        length_of_concatenated_data_blocks - end_data_index_out;

    unsigned int continuous_remaining_start_index = 0;

    if (temporal_number_of_remaining_data_blocks > 1) {

        for (unsigned int i = 0; i < temporal_number_of_remaining_data_blocks - 1; ++i) {

            angle_difference = calculate_azimuthal_angle_difference(angle_buffer[length_of_concatenated_data_blocks - i - 2],
                                                                    angle_buffer[length_of_concatenated_data_blocks - i - 1]);
            if (angle_difference > VLP16_PACKET_AZIMUTHAL_ANGLE_THRESHOLD_OF_CONTINUOUS_DATA_BLOCKS) {
                continuous_remaining_start_index = length_of_concatenated_data_blocks - i - 1;
                break;
            }

        }

    } else {
        continuous_remaining_start_index = end_data_index_out;
    }

    for (unsigned int i = continuous_remaining_start_index; i < length_of_concatenated_data_blocks; ++i) {
        memcpy(vlp16_handler->remaining_data_block_buffer[i - continuous_remaining_start_index],
               concatenated_data_blocks[i], VLP16_PACKET_DATA_BLOCK_LENGTH);
    }

    vlp16_handler->number_of_remaining_data_blocks = length_of_concatenated_data_blocks - continuous_remaining_start_index;

    free(angle_buffer);
    free(concatenated_data_blocks);

    return number_of_captured_lines;
}


static void decode_azimuthal_angles_of_dual_echo_vlp16_packet(const char **concatenated_data_blocks, unsigned int length_of_concatenated_data_blocks,
                                                              unsigned int *angle_buffer)
{
    const char *azimuthal_angle_start = NULL;
    for (unsigned int i = 0; i < length_of_concatenated_data_blocks / 2; ++i) {

        azimuthal_angle_start =
            concatenated_data_blocks[2 * i] + VLP16_PACKET_AZIMUTHAL_ANGLE_POSITION_IN_DATA_BLOCK;

        angle_buffer[i] =
            decode_unsigned_value(azimuthal_angle_start,
                                  VLP16_PACKET_AZIMUTHAL_ANGLE_LENGTH, false);
    }

    return;
}

static void decode_one_line_data_of_dual_echo_vlp16_packet(vlp16_handler_t *vlp16_handler,
                                                           unsigned int line_start_timestamp, double start_azimuthal_angle,
                                                           double one_spot_azimuthal_angle_step,
                                                           const char *first_data_buffer, const char *second_data_buffer)
{

    lidar_line_circular_buffer_t *line_data_buffer = &vlp16_handler->line_data_buffer;
    lidar_line_data_t *line_data =
        get_pointer_to_copy_lidar_line_data(line_data_buffer,
                                            LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE);

    line_data->minimum_horizontal_angle = start_azimuthal_angle;
    line_data->next_data_index = 0;

    unsigned int spot_index = 0;

    unsigned int echo_buffer_index = line_data->next_data_index;
    lidar_echo_data_t *echo_buffer = line_data->echo_buffer;

    lidar_echo_data_t *echo = &echo_buffer[echo_buffer_index];
    unsigned int echo_index = 0;

    lidar_spot_accessor_t *spot = &line_data->spot[spot_index];

    unsigned int binary_data_index = 0;

    const unsigned int number_of_spots = VLP16_PACKET_NUMBER_OF_SPOTS[vlp16_handler->decoding_packet_sensor_model];
    line_data->maximum_horizontal_angle =
        start_azimuthal_angle + one_spot_azimuthal_angle_step * (double)(number_of_spots - 1);

    if (vlp16_handler->elevation_angle_array.size() != number_of_spots) {
        make_default_vlp16_elevation_angle_array(vlp16_handler->decoding_packet_sensor_model,
                                                 vlp16_handler->elevation_angle_array,
                                                 &vlp16_handler->minimum_elevation_angle,
                                                 &vlp16_handler->maximum_elevation_angle);
    }
    line_data->minimum_elevation_angle = vlp16_handler->minimum_elevation_angle;
    line_data->maximum_elevation_angle = vlp16_handler->maximum_elevation_angle;

    unsigned int last_echo_distance = 0;
    unsigned int last_echo_intensity = 0;

    unsigned int strongest_echo_distance = 0;
    unsigned int strongest_echo_intensity = 0;

    // todo set elevation angle
    for (unsigned int spot_index = 0; spot_index < number_of_spots; ++spot_index) {
        spot = &line_data->spot[spot_index];

        last_echo_distance = decode_unsigned_value(first_data_buffer + binary_data_index,
                                                   VLP16_PACKET_DISTANCE_LENGTH, false);

        strongest_echo_distance = decode_unsigned_value(second_data_buffer + binary_data_index,
                                                        VLP16_PACKET_DISTANCE_LENGTH, false);

        binary_data_index += VLP16_PACKET_DISTANCE_LENGTH;

        last_echo_intensity = (double)decode_unsigned_value(first_data_buffer + binary_data_index,
                                                            VLP16_PACKET_REFLECTIVITY_LENGTH, false);

        strongest_echo_intensity = (double)decode_unsigned_value(second_data_buffer + binary_data_index,
                                                                 VLP16_PACKET_REFLECTIVITY_LENGTH, false);
        binary_data_index += VLP16_PACKET_REFLECTIVITY_LENGTH;

        if (last_echo_distance == strongest_echo_distance) {

            spot->number_of_echoes = 1;

            echo_index = 0;
            echo = &echo_buffer[echo_buffer_index];

            echo->index = echo_index + 1;
            echo->number_of_echoes_at_same_time = spot->number_of_echoes;

            echo->measured_time = VLP16_PACKET_ONE_LASER_FIRING_INTERVAL_USEC * (double)spot_index + (double)line_start_timestamp;
            echo->calibrated_time = echo->measured_time;

            echo->horizontal_angle = one_spot_azimuthal_angle_step * (double)(spot_index) + start_azimuthal_angle;
            echo->elevation_angle = vlp16_handler->elevation_angle_array.at(spot_index);

            echo->distance = VLP16_PACKET_DISTANCE_SCALE * (double)last_echo_distance;
            echo->intensity = (double)last_echo_intensity;

            spot->echo[echo_index] = (int)echo_buffer_index;
            ++echo_buffer_index;
            ++echo_index;

        } else {

            spot->number_of_echoes = 2;

            echo_index = 0;

            echo = &echo_buffer[echo_buffer_index];

            echo->index = echo_index + 1;
            echo->number_of_echoes_at_same_time = spot->number_of_echoes;

            echo->measured_time = VLP16_PACKET_ONE_LASER_FIRING_INTERVAL_USEC * (double)spot_index + (double)line_start_timestamp;
            echo->calibrated_time = echo->measured_time;

            echo->horizontal_angle = one_spot_azimuthal_angle_step * (double)(spot_index) + start_azimuthal_angle;
            echo->elevation_angle = vlp16_handler->elevation_angle_array.at(spot_index);

            echo->distance = VLP16_PACKET_DISTANCE_SCALE * (double)strongest_echo_distance;
            echo->intensity = (double)strongest_echo_intensity;

            spot->echo[echo_index] = (int)echo_buffer_index;
            ++echo_buffer_index;
            ++echo_index;

            echo = &echo_buffer[echo_buffer_index];

            echo->index = echo_index + 1;
            echo->number_of_echoes_at_same_time = spot->number_of_echoes;

            echo->measured_time = VLP16_PACKET_ONE_LASER_FIRING_INTERVAL_USEC * (double)spot_index + (double)line_start_timestamp;
            echo->calibrated_time = echo->measured_time;

            echo->horizontal_angle = one_spot_azimuthal_angle_step * (double)(spot_index) + start_azimuthal_angle;
            echo->elevation_angle = vlp16_handler->elevation_angle_array.at(spot_index);

            echo->distance = VLP16_PACKET_DISTANCE_SCALE * (double)last_echo_distance;
            echo->intensity = (double)last_echo_intensity;

            spot->echo[echo_index] = (int)echo_buffer_index;
            ++echo_buffer_index;
            ++echo_index;

        }

        for (; echo_index < LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES; ++echo_index) {
            spot->echo[echo_index] = LIDAR_SPOT_ACCESSOR_INVALID_ECHO_INDEX;
        }

    }
    move_copy_destination_point(line_data_buffer, 1);

    return;
}

static unsigned int decode_one_data_block_of_dual_echo_vlp16_packet(vlp16_handler_t *vlp16_handler,
                                                                    unsigned int data_block_start_timestamp,
                                                                    unsigned int start_azimuthal_angle, unsigned int end_azimuthal_angle,
                                                                    const char *first_data_buffer, const char *second_data_buffer)
{
    unsigned int captured_line_count = 0;

    const unsigned int azimuthal_angle_difference =
        calculate_azimuthal_angle_difference(start_azimuthal_angle, end_azimuthal_angle);

    const double one_spot_azimuthal_angle_step =
        VLP16_PACKET_AZIMUTHAL_ANGLE_SCALE * VLP16_PACKET_ONE_LASER_FIRING_RATIO_TO_ONE_FIRING_SEQUENCE * (double)azimuthal_angle_difference;

    const double scaled_start_azimuthal_angle =
        VLP16_PACKET_AZIMUTHAL_ANGLE_SCALE * (double)start_azimuthal_angle;

    // decode first line
    decode_one_line_data_of_dual_echo_vlp16_packet(vlp16_handler,
                                                     data_block_start_timestamp, scaled_start_azimuthal_angle,
                                                     one_spot_azimuthal_angle_step,
                                                     first_data_buffer + VLP16_PACKET_ODD_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK,
                                                     second_data_buffer + VLP16_PACKET_ODD_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK);

    ++captured_line_count;
    // decode second line
    if (vlp16_handler->decoding_packet_sensor_model == VLP16_PACKET_VLP16) {

        const double start_line_azimuthal_angle_start =
            VLP16_PACKET_AZIMUTHAL_ANGLE_SCALE * (double)(start_azimuthal_angle + (azimuthal_angle_difference / 2));

        decode_one_line_data_of_dual_echo_vlp16_packet(vlp16_handler,
                                                         data_block_start_timestamp + (unsigned int)VLP16_PACKET_LASER_FIRING_SEQUENCE_USEC,
                                                         start_line_azimuthal_angle_start,
                                                         one_spot_azimuthal_angle_step,
                                                         first_data_buffer + VLP16_PACKET_EVEN_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK,
                                                       second_data_buffer + VLP16_PACKET_EVEN_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK);
        ++captured_line_count;
    }

    return captured_line_count;
}

static unsigned int decode_dual_echo_vlp16_packet(vlp16_handler_t * vlp16_handler)
{
    const unsigned int length_of_concatenated_data_blocks =
        VLP16_PACKET_NUMBER_OF_DATA_BLOCKS + vlp16_handler->number_of_remaining_data_blocks;
    const char ** concatenated_data_blocks =
        (const char**)malloc(length_of_concatenated_data_blocks * sizeof(const char *));

    if (concatenated_data_blocks == NULL) {
        return 0;
    }

    for (unsigned int i = 0; i < vlp16_handler->number_of_remaining_data_blocks; ++i) {
        concatenated_data_blocks[i] = vlp16_handler->remaining_data_block_buffer[i];
    }

    for (unsigned int i = 0; i < VLP16_PACKET_NUMBER_OF_DATA_BLOCKS; ++i) {
        concatenated_data_blocks[i + vlp16_handler->number_of_remaining_data_blocks] = vlp16_handler->decoding_data_blocks[i];
    }

    const unsigned int half_length_of_concatenated_data_blocks = length_of_concatenated_data_blocks / 2;

    unsigned int *angle_buffer = NULL;
    angle_buffer =
        (unsigned int *) malloc(half_length_of_concatenated_data_blocks * sizeof(unsigned int));

    decode_azimuthal_angles_of_dual_echo_vlp16_packet(concatenated_data_blocks, length_of_concatenated_data_blocks,
                                                      angle_buffer);

    const unsigned int half_number_of_remaining_data_blocks = vlp16_handler->number_of_remaining_data_blocks / 2;

    unsigned int angle_difference = 0;
    unsigned int start_offset = 0;
    for (start_offset = 0; start_offset < half_number_of_remaining_data_blocks; ++start_offset) {

        angle_difference =
            calculate_azimuthal_angle_difference(angle_buffer[half_number_of_remaining_data_blocks - start_offset - 1],
                                                 angle_buffer[half_number_of_remaining_data_blocks - start_offset]);

        if (angle_difference > VLP16_PACKET_AZIMUTHAL_ANGLE_THRESHOLD_OF_CONTINUOUS_DATA_BLOCKS) {
            break;
        }
    }
    const unsigned int start_data_index = half_number_of_remaining_data_blocks - start_offset;

    unsigned int end_offset = 0;
    for (end_offset = 0; end_offset < VLP16_PACKET_HALF_NUMBER_OF_DATA_BLOCKS - 1; ++end_offset) {

        angle_difference =
            calculate_azimuthal_angle_difference(angle_buffer[half_number_of_remaining_data_blocks + end_offset],
                                                 angle_buffer[half_number_of_remaining_data_blocks + end_offset + 1]);

        if (angle_difference > VLP16_PACKET_AZIMUTHAL_ANGLE_THRESHOLD_OF_CONTINUOUS_DATA_BLOCKS) {
            break;
        }
    }
    const unsigned int end_data_index_out = half_number_of_remaining_data_blocks + end_offset;

    unsigned int start_data_block_timestamp_usec = vlp16_handler->decoding_packet_timestamp_usec;
    if (start_offset > 0) {

        unsigned int offset_of_timestamp = (unsigned int)(VLP16_PACKET_ONE_DATA_BLOCK_USEC * (double)start_offset);

        if (start_data_block_timestamp_usec < offset_of_timestamp) {
            start_data_block_timestamp_usec = VLP16_PACKET_MAXIMUM_TIMESTAMP + start_data_block_timestamp_usec - offset_of_timestamp;
        } else {
            start_data_block_timestamp_usec = start_data_block_timestamp_usec - offset_of_timestamp;
        }

    }

    unsigned int data_block_start_timestamp = start_data_block_timestamp_usec;
    unsigned int data_block_index = 2 * start_data_index;

    unsigned int number_of_captured_lines = 0;

    for (unsigned int i = start_data_index; i < end_data_index_out; ++i) {

        data_block_start_timestamp = start_data_block_timestamp_usec +
            (unsigned int)(VLP16_PACKET_ONE_DATA_BLOCK_USEC * (double)i);

        number_of_captured_lines +=
            decode_one_data_block_of_dual_echo_vlp16_packet(vlp16_handler,
                                                            data_block_start_timestamp,
                                                            angle_buffer[i], angle_buffer[i + 1],
                                                            concatenated_data_blocks[data_block_index],
                                                            concatenated_data_blocks[data_block_index + 1]);
        data_block_index += 2;

    }

    const unsigned int temporal_number_of_remaining_data_blocks =
        half_length_of_concatenated_data_blocks - end_data_index_out;

    unsigned int continuous_remaining_start_index = 0;

    if (temporal_number_of_remaining_data_blocks > 1) {

        for (unsigned int i = 0; i < temporal_number_of_remaining_data_blocks - 1; ++i) {

            angle_difference = calculate_azimuthal_angle_difference(angle_buffer[half_length_of_concatenated_data_blocks - i - 2],
                                                                    angle_buffer[half_length_of_concatenated_data_blocks - i - 1]);

            if (angle_difference > VLP16_PACKET_AZIMUTHAL_ANGLE_THRESHOLD_OF_CONTINUOUS_DATA_BLOCKS) {
                continuous_remaining_start_index = half_length_of_concatenated_data_blocks - i - 1;
                break;
            }

        }

    } else {
        continuous_remaining_start_index = end_data_index_out;
    }

    for (unsigned int i = 2 * continuous_remaining_start_index; i < length_of_concatenated_data_blocks; ++i) {
        memcpy(vlp16_handler->remaining_data_block_buffer[i - 2 * continuous_remaining_start_index],
               concatenated_data_blocks[i], VLP16_PACKET_DATA_BLOCK_LENGTH);
    }

    vlp16_handler->number_of_remaining_data_blocks = length_of_concatenated_data_blocks - 2 * continuous_remaining_start_index;

    free(angle_buffer);
    free(concatenated_data_blocks);

    return number_of_captured_lines;
}

unsigned int decode_vlp16_packet(vlp16_handler_t *vlp16_handler)
{
    if (vlp16_handler->decoding_packet_sensor_model !=
        vlp16_handler->past_packet_sensor_model) {

        allocate_memory_for_lidar_line_circular_buffer(vlp16_handler,
                                                       VLP16_PACKET_NUMBER_OF_SPOTS[vlp16_handler->decoding_packet_sensor_model],
                                                       NUMBER_OF_VLP16_PACKET_LINES_TO_STORE_MEASURED_DATA);

        clear_vlp16_remaining_data_blocks(vlp16_handler);
    }
    vlp16_handler->past_packet_sensor_model = vlp16_handler->decoding_packet_sensor_model;

    if (vlp16_handler->decoding_packet_return_mode !=
        vlp16_handler->past_packet_return_mode) {
        clear_vlp16_remaining_data_blocks(vlp16_handler);
    }
    vlp16_handler->past_packet_return_mode = vlp16_handler->decoding_packet_return_mode;

    unsigned int number_of_captured_lines = 0;

    switch (vlp16_handler->decoding_packet_return_mode) {

        case VLP16_PACKET_STRONGEST_RETURN_MODE:
        case VLP16_PACKET_LAST_RETURN_MODE:
            number_of_captured_lines =
                decode_single_echo_vlp16_packet(vlp16_handler);
            break;

        case VLP16_PACKET_DUAL_RETURN_MODE:
            number_of_captured_lines =
                decode_dual_echo_vlp16_packet(vlp16_handler);
            break;

        default:
            break;
    }

    return number_of_captured_lines;
}

bool does_no_reply_after_long_waiting_occur(const vlp16_handler_t *vlp16_handler)
{

    if (vlp16_handler->no_reply_interval_timer.TimeInterval() >
        VLP16_COMMUNICATION_HANDLER_MAXIMUM_NO_REPLY_INTERVAL_ON_AWAITING_PACKETS_USEC) {
        return true;
    }

    return false;
}

bool wait_to_receive_vlp16_measured_echoes(vlp16_handler_t *vlp16_handler,
                                           int wait_timeout_usec,
                                           unsigned int maximum_number_of_echoes,
                                           const lidar_echo_single_region_t *region,
                                           std::vector<lidar_echo_data_t> &echoes)
{
    if (echoes.size() != 0) {
        echoes.clear();
    }

    if (echoes.capacity() < maximum_number_of_echoes) {
        echoes.reserve(maximum_number_of_echoes);
    }

    bool return_bool = false;

    bool packet_received = false;
    int received_data_byte = 0;

    unsigned int captured_size = 0;

    vlp16_handler->no_reply_interval_timer.SetIntervalStart();

    TimeTheInterval timer;

    if (wait_timeout_usec < 0) {

        while (1) {

            if (echoes.size() >= maximum_number_of_echoes) {
                return_bool = true;
                break;
            }

            if (does_no_reply_after_long_waiting_occur(vlp16_handler) == true) {
                break;
            }

            packet_received =
                receive_vlp16_packet(vlp16_handler, &received_data_byte,
                                     VLP16_PACKET_LENGTH);

            if (packet_received == true) {

                captured_size =
                    decode_vlp16_packet(vlp16_handler);

                if (captured_size == 0) {
                    continue;
                }

                std::vector<const lidar_line_data_t *> captured_lines;
                get_pointers_of_latest_unused_lidar_line_data(&vlp16_handler->line_data_buffer,
                                                              captured_size, captured_lines);


                for (unsigned int i = 0; i < captured_lines.size(); ++i) {

                    if (captured_lines.at(i) == NULL) {
                        continue;
                    }
                    add_lidar_echo_data_in_a_single_region(captured_lines.at(i),
                                                           region, echoes);

                }


            }

        }

    } else {

        while (1) {

            if (echoes.size() >= maximum_number_of_echoes) {
                return_bool = true;
                break;
            }

            if (timer.TimeInterval() > (unsigned int)wait_timeout_usec) {
                break;
            }

            if (does_no_reply_after_long_waiting_occur(vlp16_handler) == true) {
                break;
            }

            packet_received =
                receive_vlp16_packet(vlp16_handler, &received_data_byte,
                                     VLP16_PACKET_LENGTH);

            if (packet_received == true) {

                captured_size =
                    decode_vlp16_packet(vlp16_handler);

                if (captured_size == 0) {
                    continue;
                }

                std::vector<const lidar_line_data_t *> captured_lines;
                get_pointers_of_latest_unused_lidar_line_data(&vlp16_handler->line_data_buffer,
                                                              captured_size, captured_lines);


                for (unsigned int i = 0; i < captured_lines.size(); ++i) {

                    if (captured_lines.at(i) == NULL) {
                        continue;
                    }
                    add_lidar_echo_data_in_a_single_region(captured_lines.at(i),
                                                           region, echoes);

                }


            }

        }

    }

    return return_bool;
}
