
#include "lidar_dataCtrl.h"

// for NULL, malloc, free
#include <stdlib.h>

// for sqrt M_PI
#include <math.h>

// for DBL_MAX
#include <float.h>

// for memcpy
#include <string.h>

// for sscanf
#include <stdio.h>

void copy_lidar_echo_data(const lidar_echo_data_t *source,
                          lidar_echo_data_t *destination)
{
    destination->index = source->index;
    destination->number_of_echoes_at_same_time =
        source->number_of_echoes_at_same_time;

    destination->horizontal_angle = source->horizontal_angle;
    destination->elevation_angle = source->elevation_angle;

    destination->measured_time = source->measured_time;
    destination->calibrated_time = source->measured_time;

    destination->distance = source->distance;
    destination->intensity = source->intensity;

    return;
}

void output_lidar_echo_data_to_stream(std::ostream &stream, const char separator,
                                      bool angle_unit_degree, const lidar_echo_data_t *echo)
{
    stream << echo->measured_time << separator
           << echo->calibrated_time << separator;

    if (angle_unit_degree == true) {
        stream << echo->horizontal_angle * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << echo->elevation_angle * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator;
    } else {
        stream << echo->horizontal_angle << separator
               << echo->elevation_angle << separator;
    }

    stream << echo->distance << separator
           << echo->intensity << separator;

    stream << echo->index << separator
           << echo->number_of_echoes_at_same_time << separator;

    return;
}

void output_lidar_echo_data_array_to_stream(std::ostream &stream, const char separator, const char line_separator,
                                            bool angle_unit_degree, const std::vector<lidar_echo_data_t> &echo_array)
{
    for (unsigned int i = 0; i < echo_array.size(); ++i) {

        output_lidar_echo_data_to_stream(stream, separator,
                                         angle_unit_degree,
                                         &echo_array.at(i));

        stream << line_separator;
    }

    return;
}

void clear_lidar_line_data(lidar_line_data_t *line)
{
    line->number_of_spots = 0;
    line->spot = NULL;

    line->start_time = 0;
    line->end_time = 0;

    line->calibrated_start_time = 0;
    line->calibrated_end_time = 0;

    line->minimum_horizontal_angle = 0.0;
    line->maximum_horizontal_angle = 0.0;

    line->minimum_elevation_angle = 0.0;
    line->maximum_elevation_angle = 0.0;

    line->echo_buffer = NULL;

    return;
}

static void clear_lidar_spot_accessor(lidar_spot_accessor_t *spot)
{
    spot->number_of_echoes = 0;

    for (unsigned int i = 0; i < LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES; ++i) {
        spot->echo[i] = LIDAR_SPOT_ACCESSOR_INVALID_ECHO_INDEX;
    }

    return;
}

static void clear_lidar_echo_data(lidar_echo_data_t *echo)
{
    echo->index = 0;
    echo->number_of_echoes_at_same_time = 0;

    echo->horizontal_angle = 0.0;
    echo->elevation_angle = 0.0;

    echo->measured_time = 0;
    echo->calibrated_time = 0;

    echo->distance = 0;
    echo->intensity = 0;

    return;
}

bool allocate_memory_for_lidar_line_data(lidar_line_data_t *line,
                                         unsigned int number_of_spots)
{
    if (number_of_spots == 0) {
        return false;
    }

    line->spot =
        (lidar_spot_accessor_t *)malloc(number_of_spots * sizeof(lidar_spot_accessor_t));

    if (line->spot == NULL) {
        return false;
    }

    const unsigned int maximum_buffer_size =
        number_of_spots * LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES;

    line->echo_buffer =
        (lidar_echo_data_t *)malloc(maximum_buffer_size * sizeof(lidar_echo_data_t));

    if (line->echo_buffer == NULL) {
        free(line->spot);
        return false;
    }

    line->number_of_spots = number_of_spots;
    line->next_data_index = 0;

    // clear accessor
    for (unsigned int i = 0; i < number_of_spots; ++i) {
        clear_lidar_spot_accessor(&line->spot[i]);
    }

    // clear echo data
    for (unsigned int i = 0; i < maximum_buffer_size; ++i) {
        clear_lidar_echo_data(&line->echo_buffer[i]);
    }

    return true;
}

bool release_memory_of_lidar_line_data(lidar_line_data_t *line)
{
    if ((line->spot == NULL) &&
        (line->echo_buffer == NULL)) {
        return false;
    }

    if (line->spot != NULL) {
        free(line->spot);
    }

    if (line->echo_buffer != NULL) {
        free(line->echo_buffer);
    }

    clear_lidar_line_data(line);

    return true;
}

bool copy_lidar_line_data(const lidar_line_data_t *source, lidar_line_data_t *destination)
{
    if (source->number_of_spots != destination->number_of_spots) {
        return false;
    }

    destination->start_time = source->start_time;
    destination->end_time = source->end_time;

    destination->calibrated_start_time = source->calibrated_start_time;
    destination->calibrated_end_time = source->calibrated_end_time;

    memcpy((void *)destination->spot, (void *)source->spot,
           sizeof(lidar_spot_accessor_t) * source->number_of_spots);

    const unsigned int maximum_buffer_size =
        source->number_of_spots * LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES;

    memcpy((void *)destination->echo_buffer, (void *)source->echo_buffer,
           sizeof(lidar_echo_data_t) * maximum_buffer_size);

    return true;
}

void initialize_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer)
{
    buffer->length = 0;

    buffer->data_start_point = NULL;
    buffer->destination_point = NULL;
    buffer->used_data_end_out_point = NULL;

    buffer->empty_buffer_length = 0;

    return;
}

void clear_memory_of_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer)
{
    for (unsigned int i = 0; i < buffer->length; ++i) {
        clear_lidar_line_data(&buffer->data_start_point[i]);
    }

    buffer->destination_point = buffer->data_start_point;
    buffer->used_data_end_out_point = buffer->data_start_point;

    buffer->empty_buffer_length = buffer->length;

    return;
}

bool allocate_memory_for_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer,
                                                    unsigned int number_of_spots, unsigned int buffer_length)
{
    if (buffer_length == 0) {
        return false;
    }

    buffer->data_start_point =
        (lidar_line_data_t *)malloc(buffer_length * sizeof(lidar_line_data_t));

    if (buffer->data_start_point == NULL) {
        return false;
    }
    buffer->length = buffer_length;

    clear_memory_of_lidar_line_circular_buffer(buffer);

    unsigned int allocated_count = 0;

    for (unsigned int i = 0; i < buffer_length; ++i) {

        if (allocate_memory_for_lidar_line_data(buffer->data_start_point + i,
                                                number_of_spots) == false) {
            break;
        }
        ++allocated_count;

    }

    if (allocated_count != buffer_length) {
        release_memory_of_lidar_line_circular_buffer(buffer);
        return false;
    }

    return true;

}

bool release_memory_of_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer)
{
    if ((buffer->data_start_point == NULL) ||
        (buffer->length == 0)) {
        return false;
    }

    for (unsigned int i = 0; i < buffer->length; ++i) {
        release_memory_of_lidar_line_data(buffer->data_start_point + i);
    }

    free(buffer->data_start_point);
    initialize_lidar_line_circular_buffer(buffer);

    return true;
}

bool is_allocated_memory_of_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer)
{
    if (buffer->length != 0) {
        return true;
    }

    return false;
}

unsigned int get_number_of_spots_of_lidar_line_circular_buffer(const lidar_line_circular_buffer_t *buffer)
{
    if (buffer->data_start_point == NULL) {
        return 0;
    }
    return buffer->data_start_point->number_of_spots;
}

unsigned int get_length_of_empty_lidar_line(const lidar_line_circular_buffer_t *buffer)
{
    return buffer->empty_buffer_length;
}

unsigned int calculate_number_of_remaining_lidar_lines(const lidar_line_circular_buffer_t *buffer)
{
    if (buffer->length < buffer->empty_buffer_length) {
        return 0;
    }

    return buffer->length - buffer->empty_buffer_length;

}

static lidar_line_data_t *calculate_shifted_pointer_in_circular_buffer(lidar_line_data_t *buffer, unsigned int buffer_length,
                                                                       lidar_line_data_t *pointer, int shift_amount)
{
    lidar_line_data_t *shifted_pointer = pointer + shift_amount;

    if (shifted_pointer < buffer) {
        shifted_pointer = shifted_pointer + buffer_length;
    }

    if (shifted_pointer >= buffer + buffer_length) {
        shifted_pointer = shifted_pointer - buffer_length;
    }

    return shifted_pointer;
}

void move_used_data_end_out_point(lidar_line_circular_buffer_t *buffer,
                                  unsigned int move_amount)
{

    buffer->used_data_end_out_point =
        calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length,
                                                     buffer->used_data_end_out_point, move_amount);

    if (buffer->empty_buffer_length + move_amount > buffer->length) {
        buffer->empty_buffer_length = buffer->length;
    } else {
        buffer->empty_buffer_length += move_amount;
    }

    return;
}

void move_copy_destination_point(lidar_line_circular_buffer_t *buffer,
                                 unsigned int move_amount)
{

    buffer->destination_point =
        calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length,
                                                     buffer->destination_point, move_amount);

    if (buffer->empty_buffer_length < move_amount) {
        buffer->empty_buffer_length = 0;
    } else {
        buffer->empty_buffer_length = buffer->empty_buffer_length - move_amount;
    }

    return;
}

static unsigned int get_capability_size(const lidar_line_circular_buffer_t *buffer,
                                        enum LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODE copy_mode,
                                        unsigned int number_of_lines)
{
    unsigned int capability_size = 0;
    unsigned int empty_size =
        get_length_of_empty_lidar_line(buffer);

    switch (copy_mode) {

        case LIDAR_LINE_CIRCULAR_BUFFER_INVALID_WRITE_MODE:
            break;

        case LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE:
            capability_size = number_of_lines;
            break;

        case LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE_PROHIBIT_PARTIAL_WRITE:
            if (number_of_lines < buffer->length) {
                capability_size = 0;
            } else {
                capability_size = number_of_lines;
            }
            break;

        case LIDAR_LINE_CIRCULAR_BUFFER_POSSIBLE_ALL_WRITE:
            if (number_of_lines > empty_size) {
                capability_size = empty_size;
            } else {
                capability_size = number_of_lines;
            }
            break;

        case LIDAR_LINE_CIRCULAR_BUFFER_PROHIBIT_PARTIAL_WRITE:
            if (number_of_lines > empty_size) {
                capability_size = 0;
            } else {
                capability_size = number_of_lines;
            }

            break;

        case NUMBER_OF_LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODES:
            break;
    }

    return capability_size;
}

unsigned int get_pointers_to_copy_lidar_line_data(lidar_line_circular_buffer_t *buffer,
                                                  enum LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODE copy_mode,
                                                  unsigned int number_of_lines,
                                                  std::vector<lidar_line_data_t *> &destination_array)
{
    const unsigned int writable_size =
        get_capability_size(buffer, copy_mode,
                            number_of_lines);

    if (destination_array.size() < writable_size) {
        destination_array.resize(writable_size);
    }

    const lidar_line_data_t *buffer_end_out = buffer->data_start_point + buffer->length;

    unsigned int remaining_length = writable_size;

    lidar_line_data_t *destination = buffer->destination_point;
    unsigned int array_index = 0;

    while(1) {

        if (destination + remaining_length > buffer_end_out) {

            const unsigned int copy_size = buffer_end_out - destination;

            for (unsigned int i = 0; i < copy_size; ++i) {
                destination_array.at(array_index + i) = destination + i;
            }
            remaining_length -= copy_size;

            array_index += copy_size;
            destination = calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length,
                                                                       destination, copy_size);

        } else {

            for (unsigned int i = 0; i < remaining_length; ++i) {
                destination_array.at(array_index + i) = destination + i;
            }

            remaining_length = 0;
            break;
        }

    }

    buffer->empty_buffer_length = buffer->empty_buffer_length - writable_size;

    return destination_array.size();
}

lidar_line_data_t *get_pointer_to_copy_lidar_line_data(lidar_line_circular_buffer_t *buffer,
                                                       enum LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODE copy_mode)
{
    if (buffer->empty_buffer_length == 0) {

        if ((copy_mode != LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE) &&
            (copy_mode != LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE_PROHIBIT_PARTIAL_WRITE)) {
            return NULL;

        }

    } else {
        buffer->empty_buffer_length = buffer->empty_buffer_length- 1;
    }
    lidar_line_data_t *pointer = buffer->destination_point;

    return pointer;
}

#include <iostream>

const lidar_line_data_t *get_pointer_of_latest_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer)
{
    const lidar_line_data_t *pointer = NULL;

    if (buffer->empty_buffer_length == buffer->length) {
        return pointer;
    }
    const int shift_amount = -1;
    pointer = calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length,
                                                           buffer->destination_point, shift_amount);
    return pointer;
}

unsigned int get_pointers_of_latest_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer,
                                                           unsigned int capture_size,
                                                           std::vector<const lidar_line_data_t *> &latest_lines)
{
    const lidar_line_data_t *pointer = NULL;

    unsigned int capturing_size = capture_size;

    if (latest_lines.size() != 0) {
        latest_lines.clear();
    }

    if (buffer->empty_buffer_length == buffer->length) {
        return 0;
    }
    const unsigned int remaining_size =
        calculate_number_of_remaining_lidar_lines(buffer);

    if (capturing_size > remaining_size) {
        capturing_size = remaining_size;
    }
    latest_lines.resize(capturing_size, NULL);

    pointer = buffer->destination_point;
    int shift_amount = 0;

    for (unsigned int i = 0; i < capturing_size; ++i) {

        shift_amount = (int)i - (int)capturing_size;

        pointer = calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length,
                                                               buffer->destination_point, shift_amount);

        latest_lines.at(i) = pointer;

    }

    return latest_lines.size();
}

const lidar_line_data_t *get_pointer_of_oldest_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer)
{
    const lidar_line_data_t *pointer = NULL;

    if (buffer->empty_buffer_length == buffer->length) {
        return pointer;
    }

    return buffer->used_data_end_out_point;
}

const lidar_line_data_t *get_buffer_end_out_of_lidar_line_data(const lidar_line_circular_buffer_t *buffer)
{
    const lidar_line_data_t *pointer = NULL;

    if (buffer->data_start_point == NULL) {
        return pointer;
    }

    pointer = buffer->data_start_point + buffer->length;

    return pointer;
}

unsigned int get_pointers_of_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer,
                                                    std::vector<const lidar_line_data_t *> &unused_data_pointers)
{
    if (unused_data_pointers.size() != 0) {
        unused_data_pointers.clear();
    }

    if (buffer->empty_buffer_length == buffer->length) {
        return 0;
    }

    const lidar_line_data_t *oldest_unused = get_pointer_of_oldest_unused_lidar_line_data(buffer);
    const lidar_line_data_t *latest_unused = get_pointer_of_latest_unused_lidar_line_data(buffer);
    const lidar_line_data_t *buffer_end_out = get_buffer_end_out_of_lidar_line_data(buffer);

    if (oldest_unused > latest_unused) {

        for (const lidar_line_data_t *pointer = oldest_unused; pointer < buffer_end_out; ++pointer) {
            unused_data_pointers.push_back(pointer);
        }

        for (const lidar_line_data_t *pointer = buffer->data_start_point; pointer <= latest_unused; ++pointer) {
            unused_data_pointers.push_back(pointer);
        }

    } else {

        for (const lidar_line_data_t *pointer = oldest_unused; pointer <= latest_unused; ++pointer) {
            unused_data_pointers.push_back(pointer);
        }

    }

    return unused_data_pointers.size();
}

void set_lidar_echo_single_region(double minimum_horizontal_angle,
                                  double maximum_horizontal_angle,
                                  double minimum_elevation_angle,
                                  double maximum_elevation_angle,
                                  double minimum_distance,
                                  double maximum_distance,
                                  double minimum_intensity,
                                  double maximum_intensity,
                                  lidar_echo_single_region_t * region)
{
    region->minimum_horizontal_angle = minimum_horizontal_angle;
    region->maximum_horizontal_angle = maximum_horizontal_angle;

    region->minimum_elevation_angle = minimum_elevation_angle;
    region->maximum_elevation_angle = maximum_elevation_angle;

    region->minimum_distance = minimum_distance;
    region->maximum_distance = maximum_distance;

    region->minimum_intensity = minimum_intensity;
    region->maximum_intensity = maximum_intensity;

    return ;
}

void set_lidar_echo_single_region_using_center_value(double center_horizontal_angle,
                                                     double horizontal_angle_torelance,
                                                     double center_elevation_angle,
                                                     double elevation_angle_torelance,
                                                     double center_distance,
                                                     double distance_torelance,
                                                     double center_intensity,
                                                     double intensity_torelance,
                                                     lidar_echo_single_region_t * region)
{
    set_lidar_echo_single_region(center_horizontal_angle - horizontal_angle_torelance,
                                 center_horizontal_angle + horizontal_angle_torelance,
                                 center_elevation_angle - elevation_angle_torelance,
                                 center_elevation_angle + elevation_angle_torelance,
                                 center_distance - distance_torelance,
                                 center_distance + distance_torelance,
                                 center_intensity - intensity_torelance,
                                 center_intensity + intensity_torelance,
                                 region);

    return;
}

void set_lidar_echo_single_region_using_direction(double minimum_horizontal_angle,
                                                  double maximum_horizontal_angle,
                                                  double minimum_elevation_angle,
                                                  double maximum_elevation_angle,
                                                  lidar_echo_single_region_t *region)
{

    region->minimum_horizontal_angle = minimum_horizontal_angle;
    region->maximum_horizontal_angle = maximum_horizontal_angle;

    region->minimum_elevation_angle = minimum_elevation_angle;
    region->maximum_elevation_angle = maximum_elevation_angle;

    region->minimum_distance = -DBL_MAX;
    region->maximum_distance = DBL_MAX;

    region->minimum_intensity = -DBL_MAX;
    region->maximum_intensity = DBL_MAX;

    return;
}

void set_lidar_echo_single_region_using_center_direction(double horizontal_angle,
                                                         double horizontal_angle_torelance,
                                                         double elevation_angle,
                                                         double elevation_angle_torelance,
                                                         lidar_echo_single_region_t *region)
{

    region->minimum_horizontal_angle = horizontal_angle - horizontal_angle_torelance;
    region->maximum_horizontal_angle = horizontal_angle + horizontal_angle_torelance;

    region->minimum_elevation_angle = elevation_angle - elevation_angle_torelance;
    region->maximum_elevation_angle = elevation_angle + elevation_angle_torelance;

    region->minimum_distance = -DBL_MAX;
    region->maximum_distance = DBL_MAX;

    region->minimum_intensity = -DBL_MAX;
    region->maximum_intensity = DBL_MAX;

    return;
}

void clear_lidar_echo_single_region(lidar_echo_single_region_t *region)
{
    set_lidar_echo_single_region(0.0, 0.0,
                                 0.0, 0.0,
                                 0.0, 0.0,
                                 0.0, 0.0, region);

    return;
}

const double LIDAR_DATA_MAXIMUM_ANGLE_RADIAN = 6.28318530718;

static bool is_angle_in_angle_interval(double start, double end, double angle)
{
    double offsetted_angle = angle;

    if (angle < start) {
        offsetted_angle = angle + LIDAR_DATA_MAXIMUM_ANGLE_RADIAN;
    } else if (angle > end) {
        offsetted_angle = angle - LIDAR_DATA_MAXIMUM_ANGLE_RADIAN;
    }

    if ((offsetted_angle < start) ||
        (offsetted_angle > end)) {
        return false;
    }

    return true;
}

bool is_lidar_echo_in_single_region(const lidar_echo_data_t *echo,
                                    const lidar_echo_single_region_t *region)
{
    if (is_angle_in_angle_interval(region->minimum_horizontal_angle,
                                   region->maximum_horizontal_angle,
                                   echo->horizontal_angle) == false) {
        return false;
    }

    if (is_angle_in_angle_interval(region->minimum_elevation_angle,
                                   region->maximum_elevation_angle,
                                   echo->elevation_angle) == false) {
        return false;
    }

    if ((echo->distance < region->minimum_distance) ||
        (echo->distance > region->maximum_distance)) {
        return false;
    }

    if ((echo->intensity < region->minimum_intensity) ||
        (echo->intensity > region->maximum_intensity)) {
        return false;
    }
    return true;

}

static bool are_intervals_intersect(double minimum1, double maximum1,
                                    double minimum2, double maximum2)
{

    if (minimum1 > maximum2) {

        return false;

    } else if (maximum1 < minimum2) {

        return false;

    } else {

        if (minimum1 > maximum1) {
            return false;
        }

        if (minimum2 > maximum2) {
            return false;
        }

    }

    return true;
}

static bool are_angle_intervals_intersect(double minimum1, double maximum1,
                                          double minimum2, double maximum2)
{
    if (minimum1 > maximum1) {
        return false;
    } else if (minimum2 > maximum2) {
        return false;
    }

    if ((maximum1 - minimum1 > LIDAR_DATA_MAXIMUM_ANGLE_RADIAN) ||
        (maximum2 - minimum2 > LIDAR_DATA_MAXIMUM_ANGLE_RADIAN)) {
        return true;
    }

    bool return_bool = false;
    return_bool = are_intervals_intersect(minimum1, maximum1,
                                          minimum2, maximum2);
    if (return_bool == false) {

        double offsetted_minimum1 = minimum1;
        double offsetted_maximum1 = maximum1;

        if (minimum1 < 0.0) {

            offsetted_minimum1 = minimum1 + LIDAR_DATA_MAXIMUM_ANGLE_RADIAN;
            offsetted_maximum1 = maximum1 + LIDAR_DATA_MAXIMUM_ANGLE_RADIAN;

        } else if (maximum1 > LIDAR_DATA_MAXIMUM_ANGLE_RADIAN) {

            offsetted_minimum1 = minimum1 - LIDAR_DATA_MAXIMUM_ANGLE_RADIAN;
            offsetted_maximum1 = maximum1 - LIDAR_DATA_MAXIMUM_ANGLE_RADIAN;

        }

        return_bool =
            are_intervals_intersect(offsetted_minimum1, offsetted_maximum1,
                                    minimum2, maximum2);

    }

    return return_bool;
}


bool are_intersect_lidar_echo_single_region_and_line(const lidar_echo_single_region_t *region,
                                                     const lidar_line_data_t *line)
{
    if (are_angle_intervals_intersect(line->minimum_horizontal_angle,
                                      line->maximum_horizontal_angle,
                                      region->minimum_horizontal_angle,
                                      region->maximum_horizontal_angle) == false) {
        return false;
    }

    if (are_angle_intervals_intersect(line->minimum_elevation_angle,
                                      line->maximum_elevation_angle,
                                      region->minimum_elevation_angle,
                                      region->maximum_elevation_angle) == false) {
        return false;
    }

    return true;
}

unsigned int copy_lidar_echo_data_in_a_single_region(const lidar_line_data_t *line,
                                                     const lidar_echo_single_region_t *region,
                                                     std::vector<lidar_echo_data_t> &echoes_in_region)
{
    if (echoes_in_region.size() != 0) {
        echoes_in_region.clear();
    }

    if (echoes_in_region.capacity() < line->number_of_spots) {
        echoes_in_region.reserve(line->number_of_spots);
    }

    if (are_intersect_lidar_echo_single_region_and_line(region, line) == false) {
        return echoes_in_region.size();
    }

    lidar_echo_data_t temporal_echo_data;

    for (unsigned int spot_index = 0; spot_index < line->number_of_spots; ++spot_index) {

        const lidar_spot_accessor_t *spot = &line->spot[spot_index];

        for (unsigned int echo_index = 0; echo_index < spot->number_of_echoes; ++echo_index) {

            if (spot->echo[echo_index] ==  LIDAR_SPOT_ACCESSOR_INVALID_ECHO_INDEX) {
                continue;
            }

            const lidar_echo_data_t *echo =
                &line->echo_buffer[spot->echo[echo_index]];

            if (is_lidar_echo_in_single_region(echo, region) == true) {

                echoes_in_region.push_back(temporal_echo_data);
                copy_lidar_echo_data(echo, &echoes_in_region.at(echoes_in_region.size() - 1));

            }

        }

    }

    return echoes_in_region.size();
}

unsigned int add_lidar_echo_data_in_a_single_region(const lidar_line_data_t *line,
                                                    const lidar_echo_single_region_t *region,
                                                    std::vector<lidar_echo_data_t> &echoes_in_region)
{
    unsigned int number_of_added_echoes = 0;

    if (are_intersect_lidar_echo_single_region_and_line(region, line) == false) {
        return number_of_added_echoes;
    }

    lidar_echo_data_t temporal_echo_data;

    for (unsigned int spot_index = 0; spot_index < line->number_of_spots; ++spot_index) {

        const lidar_spot_accessor_t *spot = &line->spot[spot_index];

        for (unsigned int echo_index = 0; echo_index < spot->number_of_echoes; ++echo_index) {

            if (spot->echo[echo_index] ==  LIDAR_SPOT_ACCESSOR_INVALID_ECHO_INDEX) {
                continue;
            }

            const lidar_echo_data_t *echo =
                &line->echo_buffer[spot->echo[echo_index]];

            if (is_lidar_echo_in_single_region(echo, region) == true) {

                echoes_in_region.push_back(temporal_echo_data);
                copy_lidar_echo_data(echo, &echoes_in_region.at(echoes_in_region.size() - 1));
                ++number_of_added_echoes;

            }

        }

    }

    return number_of_added_echoes;
}

unsigned int add_lidar_echo_data_in_each_single_region(const lidar_line_data_t *line,
                                                       const std::vector<lidar_echo_single_region_t> &regions,
                                                       std::vector< std::vector<lidar_echo_data_t> > &echoes_in_region)
{
    unsigned int total_added_size = 0;

    if (echoes_in_region.size() != regions.size()) {
        return total_added_size;
    }

    for (unsigned int region_index = 0; region_index < regions.size(); ++region_index) {

        total_added_size +=
            add_lidar_echo_data_in_a_single_region(line,
                                                   &regions.at(region_index),
                                                   echoes_in_region.at(region_index));

    }

    return total_added_size;
}

unsigned int add_lidar_echo_data_in_each_single_region(const std::vector< const lidar_line_data_t *> &line_array,
                                                       const std::vector<lidar_echo_single_region_t> &regions,
                                                       std::vector< std::vector<lidar_echo_data_t> > &echoes_in_region)
{
    unsigned int total_added_size = 0;

    if (echoes_in_region.size() != regions.size()) {
        return total_added_size;
    }

    for (unsigned int line_index = 0; line_index < line_array.size(); ++line_index) {

        total_added_size += add_lidar_echo_data_in_each_single_region(line_array.at(line_index),
                                                                      regions, echoes_in_region);

    }

    return total_added_size;
}

bool calculate_average_and_covariance_of_echoes(const std::vector<lidar_echo_data_t> &echoes,
                                                lidar_echo_data_t *average_echo,
                                                lidar_echo_data_t *variance_echo)
{
    clear_lidar_echo_data(average_echo);
    clear_lidar_echo_data(variance_echo);

    if (echoes.size() == 0) {
        return false;
    }

    const double size_inverse = 1.0 / (double)echoes.size();

    const lidar_echo_data_t *echo = NULL;

    for (unsigned int i = 0; i < echoes.size(); ++i) {

        echo = &echoes.at(i);

        average_echo->horizontal_angle +=
            echo->horizontal_angle;

        average_echo->elevation_angle +=
            echo->elevation_angle;

        average_echo->measured_time +=
            echo->measured_time;

        average_echo->calibrated_time +=
            echo->calibrated_time;

        average_echo->distance +=
            echo->distance;
        average_echo->intensity +=
            echo->intensity;

    }

    average_echo->horizontal_angle = average_echo->horizontal_angle * size_inverse;
    average_echo->elevation_angle = average_echo->elevation_angle * size_inverse;

    average_echo->measured_time = average_echo->measured_time * size_inverse;
    average_echo->calibrated_time = average_echo->calibrated_time * size_inverse;

    average_echo->distance = average_echo->distance * size_inverse;
    average_echo->intensity = average_echo->intensity * size_inverse;


    for (unsigned int i = 0; i < echoes.size(); ++i) {

        echo = &echoes.at(i);

        variance_echo->horizontal_angle +=
            (echo->horizontal_angle - average_echo->horizontal_angle) *
            (echo->horizontal_angle - average_echo->horizontal_angle);

        variance_echo->elevation_angle +=
            (echo->elevation_angle - average_echo->elevation_angle) *
            (echo->elevation_angle - average_echo->elevation_angle);

        variance_echo->measured_time +=
            (echo->measured_time - average_echo->measured_time) *
            (echo->measured_time - average_echo->measured_time);

        variance_echo->calibrated_time +=
            (echo->calibrated_time - average_echo->calibrated_time) *
            (echo->calibrated_time - average_echo->calibrated_time);

        variance_echo->distance +=
            (echo->distance - average_echo->distance) *
            (echo->distance - average_echo->distance);

        variance_echo->intensity +=
            (echo->intensity - average_echo->intensity) *
            (echo->intensity - average_echo->intensity);

    }

    variance_echo->horizontal_angle = sqrt(variance_echo->horizontal_angle * size_inverse);
    variance_echo->elevation_angle = sqrt(variance_echo->elevation_angle * size_inverse);

    variance_echo->measured_time = sqrt(variance_echo->measured_time * size_inverse);
    variance_echo->calibrated_time = sqrt(variance_echo->calibrated_time * size_inverse);

    variance_echo->distance = sqrt(variance_echo->distance * size_inverse);
    variance_echo->intensity = sqrt(variance_echo->intensity * size_inverse);

    return true;
}

enum LIDAR_ECHO_STATISTICS_DATA_INDEX {
    // echo index
    LIDAR_ECHO_INDEX_STATISTICS_INDEX = 0,

    // number of echoes
    LIDAR_NUMBER_OF_ECHOES_STATISTICS_INDEX,

    // horizontal angle
    LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX,

    // elevation angle
    LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX,

    // distance
    LIDAR_DISTANCE_STATISTICS_INDEX,

    // intensity
    LIDAR_INTENSITY_STATISTICS_INDEX,

    // x_component
    LIDAR_X_COMPONENT_STATISTICS_INDEX,
    // y_component
    LIDAR_Y_COMPONENT_STATISTICS_INDEX,
    // z_component
    LIDAR_Z_COMPONENT_STATISTICS_INDEX,

    //! number of statistics data
    NUMBER_OF_LIDAR_ECHO_STATISTICS,
};

void calculate_statistics_of_lidar_echoes(const std::vector<lidar_echo_data_t> &echoes,
                                          double one_step_angle_value,
                                          double one_step_distance_value, double one_step_intensity_value,
                                          int small_bin_threshold,
                                          double horizontal_center, double elevation_center,
                                          lidar_echo_statistics_t &statistics)
{
    std::vector<histogram_and_statistics_t *> histogram_array(NUMBER_OF_LIDAR_ECHO_STATISTICS, NULL);

    std::vector<double> one_step_value_array(NUMBER_OF_LIDAR_ECHO_STATISTICS, 0.0);
    std::vector<int> small_bin_threshold_array(NUMBER_OF_LIDAR_ECHO_STATISTICS, small_bin_threshold);

    for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {

        enum LIDAR_ECHO_STATISTICS_DATA_INDEX statistics_index =
            (enum LIDAR_ECHO_STATISTICS_DATA_INDEX) i;

        switch (statistics_index) {

            case LIDAR_ECHO_INDEX_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.echo_index;
                one_step_value_array.at(i) = 1.5;
                break;

            case LIDAR_NUMBER_OF_ECHOES_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.number_of_echoes;
                one_step_value_array.at(i) = 1.5;
                break;

            case LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.horizontal_angle;
                one_step_value_array.at(i) = one_step_angle_value;
                break;

            case LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.elevation_angle;
                one_step_value_array.at(i) = one_step_angle_value;
                break;

            case LIDAR_DISTANCE_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.distance;
                one_step_value_array.at(i) = one_step_distance_value;
                break;

            case LIDAR_INTENSITY_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.intensity;
                one_step_value_array.at(i) = one_step_intensity_value;
                break;

            case LIDAR_X_COMPONENT_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.x_component;
                one_step_value_array.at(i) = one_step_distance_value;
                break;
            case LIDAR_Y_COMPONENT_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.y_component;
                one_step_value_array.at(i) = one_step_distance_value;
                break;
            case LIDAR_Z_COMPONENT_STATISTICS_INDEX:
                histogram_array.at(i) = &statistics.z_component;
                one_step_value_array.at(i) = one_step_distance_value;
                break;

            case NUMBER_OF_LIDAR_ECHO_STATISTICS:
                break;
        }

    }

    const unsigned int number_of_echoes = echoes.size();

    std::vector< std::vector<double> > value_table(NUMBER_OF_LIDAR_ECHO_STATISTICS);

    for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {
        value_table.at(i).resize(number_of_echoes);
    }

    double horizontal_difference = 0.0;
    double elevation_difference = 0.0;

    double cosine = 0.0;

    for (unsigned int i = 0; i < number_of_echoes; ++i) {

        const lidar_echo_data_t *echo = &echoes.at(i);

        value_table.at(LIDAR_ECHO_INDEX_STATISTICS_INDEX).at(i) = echo->index;
        value_table.at(LIDAR_NUMBER_OF_ECHOES_STATISTICS_INDEX).at(i) = echo->number_of_echoes_at_same_time;

        horizontal_difference = echo->horizontal_angle - horizontal_center;

        if (horizontal_difference < -M_PI) {

            value_table.at(LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX).at(i) =
                echo->horizontal_angle + 2.0 * M_PI;

        } else if (horizontal_difference >= M_PI) {

            value_table.at(LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX).at(i) =
                echo->horizontal_angle - 2.0 * M_PI;

        } else {

            value_table.at(LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX).at(i) =
                echo->horizontal_angle;
        }

        elevation_difference = echo->elevation_angle - elevation_center;

        if (elevation_difference < -M_PI) {

            value_table.at(LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX).at(i) =
                echo->elevation_angle + 2.0 * M_PI;

        } else if (elevation_difference >= M_PI) {

            value_table.at(LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX).at(i) =
                echo->elevation_angle - 2.0 * M_PI;

        } else {

            value_table.at(LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX).at(i) =
                echo->elevation_angle;

        }

        value_table.at(LIDAR_DISTANCE_STATISTICS_INDEX).at(i) = echo->distance;
        value_table.at(LIDAR_INTENSITY_STATISTICS_INDEX).at(i) = echo->intensity;

        cosine = cos(echo->elevation_angle);

        value_table.at(LIDAR_X_COMPONENT_STATISTICS_INDEX).at(i) =
            echo->distance * cosine * cos(echo->horizontal_angle);

        value_table.at(LIDAR_Y_COMPONENT_STATISTICS_INDEX).at(i) =
            echo->distance * cosine * sin(echo->horizontal_angle);

        value_table.at(LIDAR_Z_COMPONENT_STATISTICS_INDEX).at(i) =
            echo->distance * sin(echo->elevation_angle);

    }

    calculate_region_adjusted_histogram_and_statistics_array(one_step_value_array,
                                                             small_bin_threshold_array,
                                                             value_table, histogram_array);

    return;
}

void output_lidar_echo_statistics_to_stream(std::ostream &stream, const char separator,
                                            bool angle_unit_degree, const lidar_echo_statistics_t *statistics)
{
    if (angle_unit_degree == false) {

        stream << statistics->horizontal_angle.average << separator
               << sqrt(statistics->horizontal_angle.variance) << separator
               << statistics->horizontal_angle.minimum << separator
               << statistics->horizontal_angle.maximum << separator
               << statistics->elevation_angle.average << separator
               << sqrt(statistics->elevation_angle.variance) << separator
               << statistics->elevation_angle.minimum << separator
               << statistics->elevation_angle.maximum << separator;

    } else {

        stream << statistics->horizontal_angle.average * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << sqrt(statistics->horizontal_angle.variance) * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << statistics->horizontal_angle.minimum * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << statistics->horizontal_angle.maximum * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << statistics->elevation_angle.average * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << sqrt(statistics->elevation_angle.variance) * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << statistics->elevation_angle.minimum * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
               << statistics->elevation_angle.maximum * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator;

    }

    stream << statistics->distance.average << separator
           << sqrt(statistics->distance.variance) << separator
           << statistics->distance.minimum << separator
           << statistics->distance.maximum << separator

           << statistics->intensity.average << separator
           << sqrt(statistics->intensity.variance) << separator
           << statistics->intensity.minimum << separator
           << statistics->intensity.maximum << separator

           << statistics->x_component.average << separator
           << sqrt(statistics->x_component.variance) << separator
           << statistics->x_component.minimum << separator
           << statistics->x_component.maximum << separator

           << statistics->y_component.average << separator
           << sqrt(statistics->y_component.variance) << separator
           << statistics->y_component.minimum << separator
           << statistics->y_component.maximum << separator

           << statistics->z_component.average << separator
           << sqrt(statistics->z_component.variance) << separator
           << statistics->z_component.minimum << separator
           << statistics->z_component.maximum << separator

           << statistics->number_of_echoes.average << separator
           << sqrt(statistics->number_of_echoes.variance) << separator
           << statistics->number_of_echoes.minimum << separator
           << statistics->number_of_echoes.maximum << separator
           << statistics->distance.total_number_of_data << separator;

    return;
}

void output_lidar_echo_histogram_to_stream(std::ostream &stream, const char separator, const char line_separator,
                                           bool angle_unit_degree, const lidar_echo_statistics_t *statistics)
{
    std::vector< const std::vector<double> * > large_bin_value_table(NUMBER_OF_LIDAR_ECHO_STATISTICS);
    std::vector< const std::vector<int> * > large_bin_count_table(NUMBER_OF_LIDAR_ECHO_STATISTICS);

    double step_width = 0.0;

    std::vector<double> minimum_array(NUMBER_OF_LIDAR_ECHO_STATISTICS, 0.0);
    std::vector<double> maximum_array(NUMBER_OF_LIDAR_ECHO_STATISTICS, 0.0);

    std::vector<unsigned int> total_number_array(NUMBER_OF_LIDAR_ECHO_STATISTICS, 0);

    for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {

        enum LIDAR_ECHO_STATISTICS_DATA_INDEX statistics_index =
            (enum LIDAR_ECHO_STATISTICS_DATA_INDEX) i;

        switch (statistics_index) {

            case LIDAR_ECHO_INDEX_STATISTICS_INDEX:

                minimum_array.at(i) = 0;
                maximum_array.at(i) = LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES + 1;

                large_bin_value_table.at(i) = &statistics->echo_index.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->echo_index.large_bin_count_array;
                total_number_array.at(i) = statistics->echo_index.total_number_of_data;
                break;

            case LIDAR_NUMBER_OF_ECHOES_STATISTICS_INDEX:

                minimum_array.at(i) = 0;
                maximum_array.at(i) = LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES + 1;

                large_bin_value_table.at(i) = &statistics->number_of_echoes.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->number_of_echoes.large_bin_count_array;

                total_number_array.at(i) = statistics->number_of_echoes.total_number_of_data;
                break;

            case LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX:

                step_width = calculate_step_width_of_histogram(&statistics->horizontal_angle.histogram);

                minimum_array.at(i) = statistics->horizontal_angle.minimum - step_width * 2;
                maximum_array.at(i) = statistics->horizontal_angle.maximum + step_width * 2;

                large_bin_value_table.at(i) = &statistics->horizontal_angle.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->horizontal_angle.large_bin_count_array;
                total_number_array.at(i) = statistics->horizontal_angle.total_number_of_data;
                break;

            case LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX:
                step_width = calculate_step_width_of_histogram(&statistics->elevation_angle.histogram);

                minimum_array.at(i) = statistics->elevation_angle.minimum - step_width * 2;
                maximum_array.at(i) = statistics->elevation_angle.maximum + step_width * 2;

                large_bin_value_table.at(i) = &statistics->elevation_angle.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->elevation_angle.large_bin_count_array;
                total_number_array.at(i) = statistics->elevation_angle.total_number_of_data;
                break;

            case LIDAR_DISTANCE_STATISTICS_INDEX:
                step_width = calculate_step_width_of_histogram(&statistics->distance.histogram);

                minimum_array.at(i) = statistics->distance.minimum - step_width * 2;
                maximum_array.at(i) = statistics->distance.maximum + step_width * 2;

                large_bin_value_table.at(i) = &statistics->distance.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->distance.large_bin_count_array;
                total_number_array.at(i) = statistics->distance.total_number_of_data;
                break;

            case LIDAR_INTENSITY_STATISTICS_INDEX:

                step_width = calculate_step_width_of_histogram(&statistics->intensity.histogram);

                minimum_array.at(i) = statistics->intensity.minimum - step_width * 2;

                if (minimum_array.at(i) < 0) {
                    minimum_array.at(i) = 0;
                }

                maximum_array.at(i) = statistics->intensity.maximum + step_width * 2;

                large_bin_value_table.at(i) = &statistics->intensity.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->intensity.large_bin_count_array;
                total_number_array.at(i) = statistics->intensity.total_number_of_data;
                break;

            case LIDAR_X_COMPONENT_STATISTICS_INDEX:
                step_width = calculate_step_width_of_histogram(&statistics->x_component.histogram);

                minimum_array.at(i) = statistics->x_component.minimum - step_width * 2;
                maximum_array.at(i) = statistics->x_component.maximum + step_width * 2;

                large_bin_value_table.at(i) = &statistics->x_component.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->x_component.large_bin_count_array;
                total_number_array.at(i) = statistics->x_component.total_number_of_data;
                break;

            case LIDAR_Y_COMPONENT_STATISTICS_INDEX:
                step_width = calculate_step_width_of_histogram(&statistics->y_component.histogram);

                minimum_array.at(i) = statistics->y_component.minimum - step_width * 2;
                maximum_array.at(i) = statistics->y_component.maximum + step_width * 2;

                large_bin_value_table.at(i) = &statistics->y_component.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->y_component.large_bin_count_array;
                total_number_array.at(i) = statistics->y_component.total_number_of_data;
                break;

            case LIDAR_Z_COMPONENT_STATISTICS_INDEX:
                step_width = calculate_step_width_of_histogram(&statistics->z_component.histogram);

                minimum_array.at(i) = statistics->z_component.minimum - step_width * 2;
                maximum_array.at(i) = statistics->z_component.maximum + step_width * 2;

                large_bin_value_table.at(i) = &statistics->z_component.large_bin_value_array;
                large_bin_count_table.at(i) = &statistics->z_component.large_bin_count_array;
                total_number_array.at(i) = statistics->z_component.total_number_of_data;
                break;

            case NUMBER_OF_LIDAR_ECHO_STATISTICS:
                break;
        }

    }

    unsigned int maximum_size = 0;

    for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {

        if (maximum_size < large_bin_count_table.at(i)->size()) {
            maximum_size = large_bin_count_table.at(i)->size();
        }

    }

    if (angle_unit_degree == true) {

        for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {

            enum LIDAR_ECHO_STATISTICS_DATA_INDEX statistics_index =
                (enum LIDAR_ECHO_STATISTICS_DATA_INDEX) i;

            if ((statistics_index == LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX) ||
                (statistics_index == LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX)) {

                stream << minimum_array.at(i) * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
                       << 0.0 << separator;

            } else {

                stream << minimum_array.at(i) << separator
                       << 0.0 << separator;

            }

        }
        stream << line_separator;

        for (unsigned int j = 0; j < maximum_size + 1; ++j) {

            for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {

                enum LIDAR_ECHO_STATISTICS_DATA_INDEX statistics_index =
                    (enum LIDAR_ECHO_STATISTICS_DATA_INDEX) i;

                if ((statistics_index == LIDAR_HORIZONTAL_ANGLE_STATISTICS_INDEX) ||
                    (statistics_index == LIDAR_ELEVATION_ANGLE_STATISTICS_INDEX)) {

                    if (j <  large_bin_value_table.at(i)->size()) {

                        stream << large_bin_value_table.at(i)->at(j) * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << separator
                               << (double)large_bin_count_table.at(i)->at(j) / (double)total_number_array.at(i) << separator;

                    } else if (j == large_bin_value_table.at(i)->size()){

                        stream << maximum_array.at(i) * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE
                               << separator
                               << 0.0 << separator;

                    } else {
                        stream << separator
                               << separator;
                    }

                } else {

                    if (j <  large_bin_value_table.at(i)->size()) {
                        stream << large_bin_value_table.at(i)->at(j) << separator
                               << large_bin_count_table.at(i)->at(j)  / (double)total_number_array.at(i) << separator;
                    } else if (j == large_bin_value_table.at(i)->size()) {

                        stream << maximum_array.at(i) << separator
                               << 0.0 << separator;

                    } else {
                        stream << separator
                               << separator;
                    }

                }
            }
            stream << line_separator;
        }

    } else {

        for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {

            stream << minimum_array.at(i) << separator
                   << 0.0 << separator;

        }
        stream << line_separator;

        for (unsigned int j = 0; j < maximum_size + 1; ++j) {

            for (unsigned int i = 0; i < NUMBER_OF_LIDAR_ECHO_STATISTICS; ++i) {

                if (j <  large_bin_value_table.at(i)->size()) {
                    stream << large_bin_value_table.at(i)->at(j) << separator
                           << large_bin_count_table.at(i)->at(j) / (double)total_number_array.at(i) << separator;
                } else if (j == large_bin_value_table.at(i)->size()) {

                    stream << maximum_array.at(i) << separator
                           << 0.0 << separator;

                } else {
                    stream << separator
                           << separator;
                }

            }
            stream << line_separator;
        }

    }

    return;
}

void copy_lidar_echo_statistics(const lidar_echo_statistics_t *statistics,
                                bool angle_unit_degree, std::vector<double> &value_array)
{
    if (value_array.size() != 0) {
        value_array.clear();
    }
    value_array.reserve(NUMBER_OF_LIDAR_ECHO_STATISTICS * 4);
    if (angle_unit_degree == true) {

        value_array.push_back(statistics->horizontal_angle.average *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);
        value_array.push_back(sqrt(statistics->horizontal_angle.variance) *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);
        value_array.push_back(statistics->horizontal_angle.minimum *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);
        value_array.push_back(statistics->horizontal_angle.maximum *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);

        value_array.push_back(statistics->elevation_angle.average *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);
        value_array.push_back(sqrt(statistics->elevation_angle.variance) *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);
        value_array.push_back(statistics->elevation_angle.minimum *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);
        value_array.push_back(statistics->elevation_angle.maximum *
                              LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE);


    } else {
        value_array.push_back(statistics->horizontal_angle.average);
        value_array.push_back(sqrt(statistics->horizontal_angle.variance));
        value_array.push_back(statistics->horizontal_angle.minimum);
        value_array.push_back(statistics->horizontal_angle.maximum);

        value_array.push_back(statistics->elevation_angle.average);
        value_array.push_back(sqrt(statistics->elevation_angle.variance));
        value_array.push_back(statistics->elevation_angle.minimum);
        value_array.push_back(statistics->elevation_angle.maximum);
    }

    value_array.push_back(statistics->distance.average);
    value_array.push_back(sqrt(statistics->distance.variance));
    value_array.push_back(statistics->distance.minimum);
    value_array.push_back(statistics->distance.maximum);

    value_array.push_back(statistics->intensity.average);
    value_array.push_back(sqrt(statistics->intensity.variance));
    value_array.push_back(statistics->intensity.minimum);
    value_array.push_back(statistics->intensity.maximum);

    value_array.push_back(statistics->x_component.average);
    value_array.push_back(sqrt(statistics->x_component.variance));
    value_array.push_back(statistics->x_component.minimum);
    value_array.push_back(statistics->x_component.maximum);

    value_array.push_back(statistics->y_component.average);
    value_array.push_back(sqrt(statistics->y_component.variance));
    value_array.push_back(statistics->y_component.minimum);
    value_array.push_back(statistics->y_component.maximum);

    value_array.push_back(statistics->z_component.average);
    value_array.push_back(sqrt(statistics->z_component.variance));
    value_array.push_back(statistics->z_component.minimum);
    value_array.push_back(statistics->z_component.maximum);

    value_array.push_back(statistics->number_of_echoes.average);
    value_array.push_back(statistics->number_of_echoes.variance);
    value_array.push_back(statistics->number_of_echoes.minimum);
    value_array.push_back(statistics->number_of_echoes.maximum);

    value_array.push_back(statistics->distance.total_number_of_data);

    return;
}

void clear_lidar_echo_single_region_log(lidar_echo_single_region_log_t *log)
{
    log->number_of_echoes_to_capture = 0;
    log->timeout_usec = 0;

    clear_lidar_echo_single_region(&log->region);

    log->raw_data_save = false;

    log->one_step_angle_value = 0.0;
    log->one_step_distance_value = 0.0;
    log->one_step_intensity_value = 0.0;
    log->small_bin_threshold = 0;
    log->histogram_save = false;

    log->simple_statistics_save = false;

    if (log->captured_echoes.size() != 0) {
        log->captured_echoes.clear();
    }

    return;
}

void clear_lidar_echo_data_log(lidar_echo_data_log_t *log)
{
    if (log->single_region_log_array.size() != 0) {
        log->single_region_log_array.clear();
    }

    return;
}

//! parameter tag table to handle log file of LiDAR echo measured data log
const char LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETER_TEMPLATE[NUMBER_OF_LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETERS][LIDAR_ECHO_DATA_LOG_MAXIMUM_TAG_LENGTH] =
    {"capture_times=%d", "capture_timeout_usec=%d",
     "horizon=[%lf:%lf]", "elevation=[%lf:%lf]", "distance=[%lf:%lf]", "intensity=[%lf:%lf]"};

//! type tag table to handle log file of LiDAR echo measured data log
const char LIDAR_ECHO_MEASURED_DATA_LOG_TYPES[NUMBER_OF_LIDAR_ECHO_MEASURED_DATA_LOG_TYPES][LIDAR_ECHO_DATA_LOG_MAXIMUM_TAG_LENGTH] =
    {"raw_data", "histogram:d%lf_i%lf_a%lf_th%d", "simple_statistics"};

static enum LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETER_TAG decode_string_to_measured_data_log_parameter(const std::string &parameter_string,
                                                                                                    lidar_echo_single_region_log_t &log)
{
    int tag_index = -1;
    int number_of_parameters = 1;

    int integer_parameter = 0;
    int sscanf_return_value = 0;

    for (tag_index = 0; tag_index < LIDAR_ECHO_MEASURED_DATA_LOG_HORIZONTAL_REGION; ++tag_index) {

        sscanf_return_value =
            sscanf(parameter_string.c_str(),
                   LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETER_TEMPLATE[tag_index],
                   &integer_parameter);

        if (sscanf_return_value == number_of_parameters) {

            if (tag_index == LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMES) {
                if (integer_parameter < 0) {
                    return LIDAR_ECHO_MEASURED_DATA_LOG_INVALID_TAG;
                } else {
                    log.number_of_echoes_to_capture = (unsigned int)integer_parameter;
                    return LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMES;
                }
            } else if (tag_index == LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMEOUT) {
                log.timeout_usec = integer_parameter;
                return LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMEOUT;
            }

            break;
        }

    }

    double parameter1 = 0.0;
    double parameter2 = 0.0;

    double minimum_parameter = 0.0;
    double maximum_parameter = 0.0;

    number_of_parameters = 2;

    for (tag_index = LIDAR_ECHO_MEASURED_DATA_LOG_HORIZONTAL_REGION; tag_index < NUMBER_OF_LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETERS; ++tag_index) {
        sscanf_return_value =
            sscanf(parameter_string.c_str(),
                   LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETER_TEMPLATE[tag_index],
                   &parameter1, &parameter2);

        if (sscanf_return_value == number_of_parameters) {

            if (parameter1 < parameter2) {
                minimum_parameter = parameter1;
                maximum_parameter = parameter2;
            } else {
                minimum_parameter = parameter2;
                maximum_parameter = parameter1;
            }

            if (tag_index == LIDAR_ECHO_MEASURED_DATA_LOG_HORIZONTAL_REGION) {

                log.region.minimum_horizontal_angle = minimum_parameter * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;
                log.region.maximum_horizontal_angle = maximum_parameter * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

                return LIDAR_ECHO_MEASURED_DATA_LOG_HORIZONTAL_REGION;

            } else if (tag_index == LIDAR_ECHO_MEASURED_DATA_LOG_ELEVATION_REGION) {

                log.region.minimum_elevation_angle = minimum_parameter * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;
                log.region.maximum_elevation_angle = maximum_parameter * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

                return LIDAR_ECHO_MEASURED_DATA_LOG_ELEVATION_REGION;

            } else if (tag_index == LIDAR_ECHO_MEASURED_DATA_LOG_DISTANCE_LIMIT) {

                log.region.minimum_distance = minimum_parameter;
                log.region.maximum_distance = maximum_parameter;

                return LIDAR_ECHO_MEASURED_DATA_LOG_DISTANCE_LIMIT;

            } else if (tag_index == LIDAR_ECHO_MEASURED_DATA_LOG_INTENSITY_LIMIT) {

                log.region.minimum_intensity = minimum_parameter;
                log.region.maximum_intensity = maximum_parameter;

                return LIDAR_ECHO_MEASURED_DATA_LOG_INTENSITY_LIMIT;
            }

            break;
        }

    }

    number_of_parameters = 0;
    double parameter3 = 0.0;

    for (int log_type_tag = 0; log_type_tag < NUMBER_OF_LIDAR_ECHO_MEASURED_DATA_LOG_TYPES; ++log_type_tag) {

        if (log_type_tag == LIDAR_ECHO_MEASURED_DATA_LOG_HISTOGRAM) {

            number_of_parameters = 4;
            sscanf_return_value =
                sscanf(parameter_string.c_str(),
                       LIDAR_ECHO_MEASURED_DATA_LOG_TYPES[log_type_tag],
                       &parameter1, &parameter2, &parameter3, &integer_parameter);

            if (sscanf_return_value == number_of_parameters) {

                if ((parameter1 > 0.0) &&
                    (parameter2 > 0.0) &&
                    (parameter3 > 0.0) &&
                    (integer_parameter >= 0)) {
                    log.one_step_distance_value = parameter1;
                    log.one_step_intensity_value = parameter2;
                    log.one_step_angle_value = parameter3 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

                    log.small_bin_threshold = integer_parameter;
                    log.histogram_save = true;

                }
            }

        } else {

            if (strcmp(parameter_string.c_str(),
                       LIDAR_ECHO_MEASURED_DATA_LOG_TYPES[log_type_tag]) == 0) {

                if (log_type_tag == LIDAR_ECHO_MEASURED_DATA_LOG_RAW) {
                    log.raw_data_save = true;
                } else if (log_type_tag == LIDAR_ECHO_MEASURED_DATA_LOG_SIMPLE_STATISTICS) {
                    log.simple_statistics_save = true;
                }
            }

        }
    }

    return LIDAR_ECHO_MEASURED_DATA_LOG_INVALID_TAG;
}

static bool parse_string_array_to_lidar_echo_measured_data_log_parameter(const std::vector<std::string> &string_array,
                                                                         lidar_echo_data_log_t &log)
{
    std::vector<bool> parameter_set_flags(NUMBER_OF_LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETERS, false);

    lidar_echo_single_region_log_t single_region_log;
    clear_lidar_echo_single_region_log(&single_region_log);

    enum LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETER_TAG decoded_parameter_index =
        LIDAR_ECHO_MEASURED_DATA_LOG_INVALID_TAG;

    for (unsigned int string_index = 0; string_index < string_array.size(); ++string_index) {

        decoded_parameter_index =
            decode_string_to_measured_data_log_parameter(string_array.at(string_index),
                                                         single_region_log);

        if (decoded_parameter_index != LIDAR_ECHO_MEASURED_DATA_LOG_INVALID_TAG) {
            parameter_set_flags.at((int)decoded_parameter_index) = true;
        }

    }

    if ((parameter_set_flags.at(LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMES) == true) &&
        (parameter_set_flags.at(LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMEOUT) == true) &&
        (parameter_set_flags.at(LIDAR_ECHO_MEASURED_DATA_LOG_HORIZONTAL_REGION) == true) &&
        (parameter_set_flags.at(LIDAR_ECHO_MEASURED_DATA_LOG_ELEVATION_REGION) == true)) {

        if (parameter_set_flags.at(LIDAR_ECHO_MEASURED_DATA_LOG_DISTANCE_LIMIT) == false) {
            single_region_log.region.minimum_distance = -DBL_MAX;
            single_region_log.region.maximum_distance = DBL_MAX;
        }

        if (parameter_set_flags.at(LIDAR_ECHO_MEASURED_DATA_LOG_INTENSITY_LIMIT) == false) {
            single_region_log.region.minimum_intensity = -DBL_MAX;
            single_region_log.region.maximum_intensity = DBL_MAX;
        }

        log.single_region_log_array.push_back(single_region_log);

        return true;
    }

    return false;
}

static enum LIDAR_ECHO_DATA_LOG_TAG parse_string_array_to_lidar_echo_data_log_parameter(const std::vector<std::string> &string_array,
                                                                                        lidar_echo_data_log_t &log)
{
    int log_tag_index = -1;

    for (log_tag_index = 0; log_tag_index < NUMBER_OF_LIDAR_ECHO_DATA_LOG_TAGS; ++log_tag_index) {

        if (strcmp(string_array.at(LIDAR_ECHO_DATA_LOG_TAG_INDEX_IN_STRING_ARRAY).c_str(),
                   LIDAR_ECHO_DATA_LOG_TAG[log_tag_index]) == 0) {
            break;
        }

    }

    if (log_tag_index == NUMBER_OF_LIDAR_ECHO_DATA_LOG_TAGS) {
        return LIDAR_ECHO_DATA_LOG_INVALID_TAG;
    }

    enum LIDAR_ECHO_DATA_LOG_TAG log_tag = (enum LIDAR_ECHO_DATA_LOG_TAG)log_tag_index;

    switch (log_tag) {

    case LIDAR_ECHO_DATA_LOG_INVALID_TAG:
    case NUMBER_OF_LIDAR_ECHO_DATA_LOG_TAGS:
        log_tag = LIDAR_ECHO_DATA_LOG_INVALID_TAG;
        break;

    case LIDAR_ECHO_MEASURED_DATA_LOG:
        if (parse_string_array_to_lidar_echo_measured_data_log_parameter(string_array, log) == false) {
            log_tag = LIDAR_ECHO_DATA_LOG_INVALID_TAG;
        }
        break;
    }

    return log_tag;
}

void parse_string_table_to_lidar_echo_data_log_parameter(const std::vector< std::vector<std::string> > &string_table,
                                                         lidar_echo_data_log_t &log)
{
    for (unsigned int array_index = 0; array_index < string_table.size(); ++array_index) {

        parse_string_array_to_lidar_echo_data_log_parameter(string_table.at(array_index),
                                                            log);

    }

    return;
}

void output_lidar_echo_data_log_parameter(std::ostream &stream, const lidar_echo_data_log_t &log)
{
    stream << "Number of single region log parameters is " << log.single_region_log_array.size() << "\n";

    if (log.single_region_log_array.size() == 0) {
        return;
    }
    stream << "\n";

    for (unsigned int i = 0; i < log.single_region_log_array.size(); ++i) {

        stream << i << " th single region parameter:\n";
        const lidar_echo_single_region_log_t *single_region_log =
            &log.single_region_log_array.at(i);

        const lidar_echo_single_region_t *region = &single_region_log->region;

        stream << "  "
               << "capture_times     =  " << single_region_log->number_of_echoes_to_capture << "\n"
               << "  ";
        if (single_region_log->timeout_usec < 0) {
            stream << "timeout           = none \n";
        } else {
            stream << "timeout           =  " << 0.001 * 0.001 * (double)single_region_log->timeout_usec << " [sec]\n";
        }
        stream << "  "
               << "horizontal        =  ["
               << region->minimum_horizontal_angle * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << ":"
               << region->maximum_horizontal_angle * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << "] [degree]\n"
               << "  "
               << "elevation         =  ["
               << region->minimum_elevation_angle * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << ":"
               << region->maximum_elevation_angle * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << "] [degree]\n";

        if ((region->minimum_distance != -DBL_MAX) &&
            (region->maximum_distance != DBL_MAX)) {

            stream << "  "
                   << "distance          =  ["
                   << region->minimum_distance << ":"
                   << region->maximum_distance << "]\n";

        } else if ((region->minimum_distance == -DBL_MAX) &&
                   (region->maximum_distance == DBL_MAX)) {

            stream << "  "
                   << "distance          =  no limit\n";

        } else if (region->minimum_distance == -DBL_MAX) {
            stream << "  "
                   << "distance          =  [lower no limit:"
                   << region->maximum_distance << "]\n";
        } else {
            stream << "  "
                   << "distance          =  ["
                   << region->minimum_distance << ":"
                   << "upper no limit]\n";
        }

        if ((region->minimum_intensity != -DBL_MAX) &&
            (region->maximum_intensity != DBL_MAX)) {

            stream << "  "
                   << "intensity         =  ["
                   << region->minimum_intensity << ":"
                   << region->maximum_intensity << "]\n";

        } else if ((region->minimum_intensity == -DBL_MAX) &&
                   (region->maximum_intensity == DBL_MAX)) {

            stream << "  "
                   << "intensity         =  no limit\n";

        } else if (region->minimum_intensity == -DBL_MAX) {
            stream << "  "
                   << "intensity         =  [lower no limit:"
                   << region->maximum_intensity << "]\n";
        } else {
            stream << "  "
                   << "intensity         =  ["
                   << region->minimum_intensity << ":"
                   << "upper no limit]\n";
        }

        if (single_region_log->raw_data_save == true) {
            stream << "  "
                   << "raw data save     =  true\n";
        }

        if (single_region_log->histogram_save == true) {
            stream << "  "
                   << "histogram save    =  true\n"
                   << "    "
                   << "angle step          : " << single_region_log->one_step_angle_value * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE << " [degree]\n"
                   << "    "
                   << "distance step       : " << single_region_log->one_step_distance_value << "\n"
                   << "    "
                   << "intensity step      : " << single_region_log->one_step_intensity_value << "\n"
                   << "    "
                   << "small bin threshold : " << single_region_log->small_bin_threshold << "\n";
        }

        if (single_region_log->simple_statistics_save == true) {
            stream << "  "
                   << "simple statistics = true\n";
        }

    }
    stream << "\n";

    return;
}
