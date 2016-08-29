#ifndef LIDAR_DATA_CONTROL_H
#define LIDAR_DATA_CONTROL_H
/*!
  \file
  \brief functions to control measured data by LiDAR
  \author Kiyoshi MATSUO
  $Id$
*/

// include for statistics
#include "histogramCtrl.h"

// include for vector of stl
#include <vector>

// include for ostream
#include <ostream>

//! coefficient to convert degree to radian 3.1415926535 / 180.0
#define LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN 0.01745329251

//! coefficient to convert degree to radian 180.0 / 3.1415926535
#define LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_RADIAN_TO_DEGREE 57.2957795147

//! structure for one echo data measured by LiDAR
struct lidar_echo_data_t {

    //! echo index
    unsigned int index;

    //! number of measured echoes
    unsigned int number_of_echoes_at_same_time;

    //! measurement direction (horizontal angle)
    double horizontal_angle;

    //! measurement direction (elevation angle)
    double elevation_angle;

    //! measured sensor time
    unsigned int measured_time;

    //! calibrated time
    unsigned int calibrated_time;

    //! measured distance
    unsigned int distance;

    //! measured intensity (or reflectivity)
    unsigned int intensity;
};

/*!
  \brief function to copy lidar echo data
*/
extern void copy_lidar_echo_data(const lidar_echo_data_t *source,
                                 lidar_echo_data_t *destination);


/*!
  \brief function to output lidar echo data to stream
*/
extern void output_lidar_echo_data_to_stream(std::ostream &stream, const char separator,
                                             bool angle_unit_degree, const lidar_echo_data_t *echo);

/*!
  \brief function to output lidar echo data array to stream
*/
extern void output_lidar_echo_data_array_to_stream(std::ostream &stream, const char separator, const char line_separator,
                                                   bool angle_unit_degree, const std::vector<lidar_echo_data_t> &echo_array);

//! constants for LiDAR data control
enum LIDAR_SPOT_ACESSOR_CONSTANT {
    //! maximum number of echoes
    LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES = 3,

    //! invalid index of no echo
    LIDAR_SPOT_ACCESSOR_INVALID_ECHO_INDEX = -1,

};

//! structure of echo spot data accessor (spot data consists of some echoes)
struct lidar_spot_accessor_t {

    //! number of echoes
    unsigned int number_of_echoes;

    //! index of each echo in echo_buffer
    int echo[LIDAR_SPOT_ACCESSOR_MAXIMUM_NUMBER_OF_ECHOES];

};

//! struct for measured data on one measurement sequence
struct lidar_line_data_t {

    //! number of measurement
    unsigned int number_of_spots;

    //! accessor buffer
    lidar_spot_accessor_t *spot;

    //! measured time at line start
    unsigned int start_time;
    //! measured time at line end
    unsigned int end_time;

    //! calibrated time at line start
    unsigned int calibrated_start_time;
    //! calibrated time at line end
    unsigned int calibrated_end_time;

    //! minimum horizontal angle
    double minimum_horizontal_angle;
    //! maximum horizontal angle
    double maximum_horizontal_angle;

    //! minimum elevation angle
    double minimum_elevation_angle;
    //! maximum elevation angle
    double maximum_elevation_angle;

    //! next data index (destination of copy)
    unsigned int next_data_index;

    //! data buffer
    lidar_echo_data_t *echo_buffer;
};

/*!
  \brief function to clear line data
  \attention this function should be used for initialize line data
  \attention this function set pointers null without checking whether memories are allocated
*/
extern void clear_lidar_line_data(lidar_line_data_t *line);

/*!
  \brief function to allocate memory for line data
  \attention line should be cleared before it is set on this function
  \attention this function does not check line data is initialized by clear_lidar_line_data
*/
extern bool allocate_memory_for_lidar_line_data(lidar_line_data_t *line,
                                                unsigned int number_of_spots);

/*!
  \brief function to release memory of line data
*/
extern bool release_memory_of_lidar_line_data(lidar_line_data_t *line);

/*!
  \brief function to copy lidar line data
  \attention this function returns false if spot sizes are not equivalent
*/
extern bool copy_lidar_line_data(const lidar_line_data_t *source, lidar_line_data_t *destination);

//! structure for circular buffer of line data
struct lidar_line_circular_buffer_t {

    //! buffer size
    unsigned int length;

    //! remaining empty buffer
    unsigned int empty_buffer_length;

    //! buffer pointer
    lidar_line_data_t *data_start_point;

    //! copy destination point
    lidar_line_data_t *destination_point;

    //! data used endout point
    lidar_line_data_t *used_data_end_out_point;

};

/*!
  \brief function to initialize circular buffer of lines measured by LiDAR
  \attention this function should be used to initialize buffer
  \attention this function set data_start_point, destination_point, and used_data_end_out_point are NULL
*/
extern void initialize_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to clear memory of lidar line circular buffer
*/
extern void clear_memory_of_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to allocate memory for circular line buffer
  \attention this function does not check whether buffer is initialized
*/
extern bool allocate_memory_for_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer,
                                                           unsigned int number_of_spots, unsigned int buffer_length);

/*!
  \brief function to release memory of lidar line circular buffer
*/
extern bool release_memory_of_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to evaluate whether lidar line circular buffer is allocated
  \attention this function does not work if buffer is not initialized
*/
extern bool is_allocated_memory_of_lidar_line_circular_buffer(lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to get number of spots of lidar line circular
*/
extern unsigned int get_number_of_spots_of_lidar_line_circular_buffer(const lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to get length of empty data length
*/
extern unsigned int get_length_of_empty_lidar_line(const lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to calculate number of remaining lidar line
*/
extern unsigned int calculate_number_of_remaining_lidar_lines(const lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to move used data end out pointer
  \attention this function does not work if used_data_end_out_point is not in buffer or move_amount size is larget than buffer_length
*/
extern void move_used_data_end_out_point(lidar_line_circular_buffer_t *buffer,
                                         unsigned int move_amount);

/*!
  \brief function to move copy-destination pointer
  \attention this function does not work if destination_point is not in buffer or move_amount size is larget than buffer_length
*/
extern void move_copy_destination_point(lidar_line_circular_buffer_t *buffer,
                                        unsigned int move_amount);

//! write mode for lidar line circular buffer
enum LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODE {

    //! invalid write mode
    LIDAR_LINE_CIRCULAR_BUFFER_INVALID_WRITE_MODE = -1,

    //! write all data
    LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE = 0,

    //! write all data with prohibiting partial write
    LIDAR_LINE_CIRCULAR_BUFFER_OVERWRITE_PROHIBIT_PARTIAL_WRITE,

    //! write possible data without overwrite
    LIDAR_LINE_CIRCULAR_BUFFER_POSSIBLE_ALL_WRITE,

    //! write only one-set data
    LIDAR_LINE_CIRCULAR_BUFFER_PROHIBIT_PARTIAL_WRITE,

    //! number of write modes
    NUMBER_OF_LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODES,

};

/*!
  \brief function to get pointers to copy lidar line data
*/
extern unsigned int get_pointers_to_copy_lidar_line_data(lidar_line_circular_buffer_t *buffer,
                                                         enum LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODE copy_mode,
                                                         unsigned int number_of_lines,
                                                         std::vector<lidar_line_data_t *> &destination_array);

/*!
  \brief function to get pointer to copy lidar line data
*/
extern lidar_line_data_t *get_pointer_to_copy_lidar_line_data(lidar_line_circular_buffer_t *buffer,
                                                              enum LIDAR_LINE_CIRCULAR_BUFFER_WRITE_MODE copy_mode);

/*!
  \brief function to get latest unused data pointer
*/
extern const lidar_line_data_t *get_pointer_of_latest_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to get latest unused data pointers
*/
extern unsigned int get_pointers_of_latest_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer,
                                                                  unsigned int capture_size,
                                                                  std::vector<const lidar_line_data_t *> &latest_lines);

/*!
  \brief function to get oldest unused data pointer
*/
extern const lidar_line_data_t *get_pointer_of_oldest_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to get buffer end out pointer
*/
extern const lidar_line_data_t *get_buffer_end_out_of_lidar_line_data(const lidar_line_circular_buffer_t *buffer);

/*!
  \brief function to get unused data pointers
*/
extern unsigned int get_pointers_of_unused_lidar_line_data(const lidar_line_circular_buffer_t *buffer,
                                                           std::vector<const lidar_line_data_t *> &unused_data_pointers);

//! structure for region of lidar data
struct lidar_echo_single_region_t {

    //! minimum horizontal angle
    double minimum_horizontal_angle;
    //! maximum horizontal angle
    double maximum_horizontal_angle;

    //! minimum elevation angle
    double minimum_elevation_angle;
    //! maximum elevation angle;
    double maximum_elevation_angle;

    //! minimum distance
    double minimum_distance;
    //! maximum distance
    double maximum_distance;

    //! minimum intensity
    double minimum_intensity;
    //! maximum intensity
    double maximum_intensity;

};

/*!
  \brief function to set lidar echo single region
*/
extern void set_lidar_echo_single_region(double minimum_horizontal_angle,
                                         double maximum_horizontal_angle,
                                         double minimum_elevation_angle,
                                         double maximum_elevation_angle,
                                         double minimum_distance,
                                         double maximum_distance,
                                         double minimum_intensity,
                                         double maximum_intensity,
                                         lidar_echo_single_region_t * region);

/*!
  \brief function to set lidar echo single region using center value
  \attention this function set [center-torelance, center+torelance]
*/
extern void set_lidar_echo_single_region_using_center_value(double center_horizontal_angle,
                                                            double horizontal_angle_torelance,
                                                            double center_elevation_angle,
                                                            double elevation_angle_torelance,
                                                            double center_distance,
                                                            double distance_torelance,
                                                            double center_intensity,
                                                            double intensity_torelance,
                                                            lidar_echo_single_region_t * region);

/*!
  \brief function to set lidar echo single region using only angular conditions
  \attention this function set distance condition [-DBL_MAX, DBL_MAX] and intensity condition [-DBL_MAX, DBL_MAX]
*/
extern void set_lidar_echo_single_region_using_direction(double minimum_horizontal_angle,
                                                         double maximum_horizontal_angle,
                                                         double minimum_elevation_angle,
                                                         double maximum_elevation_angle,
                                                         lidar_echo_single_region_t *region);

/*!
  \brief function to set lidar echo single region using only angular conditions
  \attention this function set distance condition [-DBL_MAX, DBL_MAX] and intensity condition [-DBL_MAX, DBL_MAX]
*/
extern void set_lidar_echo_single_region_using_center_direction(double horizontal_angle,
                                                                double horizontal_angle_torelance,
                                                                double elevation_angle,
                                                                double elevation_angle_torelance,
                                                                lidar_echo_single_region_t *region);

/*!
  \brief function to clear lidar echo single region
*/
extern void clear_lidar_echo_single_region(lidar_echo_single_region_t *region);

/*!
  \brief function to evaluate whether lidar echo is in a single region
*/
extern bool is_lidar_echo_in_single_region(const lidar_echo_data_t *echo,
                                           const lidar_echo_single_region_t *region);

/*!
  \brief function to evaluate whether lidar echo single region is intersected with line
  \attention this function check only horizontal and elevation angle
*/
extern bool are_intersect_lidar_echo_single_region_and_line(const lidar_echo_single_region_t *region,
                                                            const lidar_line_data_t *line);

/*!
  \brief function to copy lidar echo data in a single region
*/
extern unsigned int copy_lidar_echo_data_in_a_single_region(const lidar_line_data_t *line,
                                                            const lidar_echo_single_region_t *region,
                                                            std::vector<lidar_echo_data_t> &echoes_in_region);

/*!
  \brief function to add lidar echo data in a single region
*/
extern unsigned int add_lidar_echo_data_in_a_single_region(const lidar_line_data_t *line,
                                                           const lidar_echo_single_region_t *region,
                                                           std::vector<lidar_echo_data_t> &echoes_in_region);

/*!
  \brief function to add lidar echo data of each single region
  \attention this funtion returns total size of echoes which are added
  \attention this function does not add echoes if size of echoes_in_region and regions are not equal.
*/
extern unsigned int add_lidar_echo_data_in_each_single_region(const lidar_line_data_t *line,
                                                              const std::vector<lidar_echo_single_region_t> &regions,
                                                              std::vector< std::vector<lidar_echo_data_t> > &echoes_in_region);

/*!
  \brief function to add lidar echo data of each single region
  \attention this funtion returns total size of echoes which are added
  \attention this function does not add echoes if size of echoes_in_region and regions are not equal.
*/
extern unsigned int add_lidar_echo_data_in_each_single_region(const std::vector< const lidar_line_data_t *> &line_array,
                                                              const std::vector<lidar_echo_single_region_t> &regions,
                                                              std::vector< std::vector<lidar_echo_data_t> > &echoes_in_region);

/*!
  \brief function to calculate average and covariance of echoes
*/
extern bool calculate_average_and_covariance_of_echoes(const std::vector<lidar_echo_data_t> &echoes,
                                                       lidar_echo_data_t *average_echo,
                                                       lidar_echo_data_t *variance_echo);


//! structure for statistics of lidar echoes
struct lidar_echo_statistics_t {

    //! echo index (single value total statistics)
    histogram_and_statistics_t echo_index;

    //! number of echoes (single value total statistics)
    histogram_and_statistics_t number_of_echoes;

    //! horizontal angle center
    double horizontal_angle_calculation_center;

    //! horizontal angle (single value total statistics)
    histogram_and_statistics_t horizontal_angle;

    //! elevation angle center
    double elevation_angle_calculation_center;

    //! elevation angle (single value total statistics)
    histogram_and_statistics_t elevation_angle;

    //! distance (single value total statistics)
    histogram_and_statistics_t distance;

    //! intensity (single value total statistics)
    histogram_and_statistics_t intensity;

    //! x_component (single value total statistics)
    histogram_and_statistics_t x_component;
    //! y_component (single value total statistics)
    histogram_and_statistics_t y_component;
    //! z_component (single value total statistics)
    histogram_and_statistics_t z_component;

    //! single value clustering results

    //! multi value total statistics

};

/*!
  \brief function to calculate statistics of lidar echoes
*/
extern void calculate_statistics_of_lidar_echoes(const std::vector<lidar_echo_data_t> &echoes,
                                                 double one_step_angle_value,
                                                 double one_step_distance_value, double one_step_intensity_value,
                                                 int small_bin_threshold,
                                                 double horizontal_center, double elevation_center,
                                                 lidar_echo_statistics_t &statistics);

/*!
  \brief function to output statistics to stream
*/
extern void output_lidar_echo_statistics_to_stream(std::ostream &stream, const char separator,
                                                   bool angle_unit_degree, const lidar_echo_statistics_t *statistics);

/*!
  \brief function to output histogram to stream
*/
extern void output_lidar_echo_histogram_to_stream(std::ostream &stream, const char separator, const char line_separator,
                                                  bool angle_unit_degree, const lidar_echo_statistics_t *statistics);

/*!
  \brief function to copy statistics information
*/
extern void copy_lidar_echo_statistics(const lidar_echo_statistics_t *statistics,
                                       bool angle_unit_degree, std::vector<double> &value_array);

//! tags of log file of LiDAR echo data
enum LIDAR_ECHO_DATA_LOG_TAG {

    //! invalid tag
    LIDAR_ECHO_DATA_LOG_INVALID_TAG = -1,

    //! measured data log (raw data / multi spots statistics)
    LIDAR_ECHO_MEASURED_DATA_LOG,

    //! number of tags of LiDAR echo data
    NUMBER_OF_LIDAR_ECHO_DATA_LOG_TAGS,
};

//! constants to handle log file of LiDAR echo data
enum LIDAR_ECHO_DATA_LOG_CONSTANT {

    //! index of tag in string array
    LIDAR_ECHO_DATA_LOG_TAG_INDEX_IN_STRING_ARRAY = 0,

    //! maximum length of tag of log file
    LIDAR_ECHO_DATA_LOG_MAXIMUM_TAG_LENGTH = 64,

};

//! tag table to handle log file of LiDAR echo data
const char LIDAR_ECHO_DATA_LOG_TAG[NUMBER_OF_LIDAR_ECHO_DATA_LOG_TAGS][LIDAR_ECHO_DATA_LOG_MAXIMUM_TAG_LENGTH] =
    {"lidar_echo_measured_data"};

//! tags of parameters of measured data log of LiDAR echo data
enum LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETER_TAG {

    //! invalid parameter tag
    LIDAR_ECHO_MEASURED_DATA_LOG_INVALID_TAG = -1,

    //! number of echoes to capture
    LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMES,

    //! timeout usec to capture
    LIDAR_ECHO_MEASURED_DATA_LOG_CAPTURE_TIMEOUT,

    //! horizontal angle
    LIDAR_ECHO_MEASURED_DATA_LOG_HORIZONTAL_REGION,

    //! elevation angle
    LIDAR_ECHO_MEASURED_DATA_LOG_ELEVATION_REGION,

    //! distance
    LIDAR_ECHO_MEASURED_DATA_LOG_DISTANCE_LIMIT,
    //! intensity
    LIDAR_ECHO_MEASURED_DATA_LOG_INTENSITY_LIMIT,

    //! number of parameter tags
    NUMBER_OF_LIDAR_ECHO_MEASURED_DATA_LOG_PARAMETERS,

};

//! tags of types of measured data log of LiDAR echo data
enum LIDAR_ECHO_MEASURED_DATA_LOG_TYPE_TAG {

    //! invalid type tag
    LIDAR_ECHO_MEASURED_DATA_LOG_INVALID_TYPE = -1,

    //! raw data
    LIDAR_ECHO_MEASURED_DATA_LOG_RAW,

    //! histogram
    LIDAR_ECHO_MEASURED_DATA_LOG_HISTOGRAM,

    //! simple statistics (average, variance)
    LIDAR_ECHO_MEASURED_DATA_LOG_SIMPLE_STATISTICS,

    //! number of type tags
    NUMBER_OF_LIDAR_ECHO_MEASURED_DATA_LOG_TYPES,

};

//! structure for log file of LiDAR echo data in single region
struct lidar_echo_single_region_log_t {

    //! number of echoes to capture
    unsigned int number_of_echoes_to_capture;

    //! timeout to capture echoes
    int timeout_usec;

    //! region parameter
    lidar_echo_single_region_t region;

    //! raw data save
    bool raw_data_save;

    //! angle bin size of histogram
    double one_step_angle_value;
    //! distance bin size of histogram
    double one_step_distance_value;
    //! intensity bin size of histogram
    double one_step_intensity_value;
    //! small bin threshold
    int small_bin_threshold;
    bool histogram_save;

    //! simple statistics data save
    bool simple_statistics_save;

    //! captured echoes
    std::vector<lidar_echo_data_t> captured_echoes;

};

/*!
  \brief function to clear parameters of single region log
*/
extern void clear_lidar_echo_single_region_log(lidar_echo_single_region_log_t *log);

//! structure for log file of LiDAR echo data
struct lidar_echo_data_log_t {

    //! array of parameters of single regions
    std::vector<lidar_echo_single_region_log_t> single_region_log_array;

};

/*!
  \brief function to clear parameters of LiDAR echo data log
*/
extern void clear_lidar_echo_data_log(lidar_echo_data_log_t *log);


/*!
  \brief function to parse string table to lidar echo data log parameter
*/
extern void parse_string_table_to_lidar_echo_data_log_parameter(const std::vector< std::vector<std::string> > &string_table,
                                                                lidar_echo_data_log_t &log);

/*!
  \brief function to output lidar echo data log parameter to stream
*/
extern void output_lidar_echo_data_log_parameter(std::ostream &stream, const lidar_echo_data_log_t &log);

#endif // LIDAR_DATA_CONTROL_H
