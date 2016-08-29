/*!
  \file
  \brief test program of communication with VLP-16
  \author Kiyoshi MATSUO
  $Id: 2016-07-12 23:33 +0900 $
*/

#include "vlp16Ctrl.h"

// for memset
#include <string.h>

// for cos/sin
#include <math.h>

#include <iostream>

using namespace std;

//! default parameter file name
const char DEFAULT_PARAMETER_FILE_NAME[] = "vlp16_command_test_parameter.csv";

//! default sensor ip address
const char DEFAULT_SENSOR_IP_ADDRESS[] = "192.168.0.20";
//! default sensor port number
const char DEFAULT_SENSOR_PORT_NUMBER[] = "2368";

//! default reception ip address
const char DEFAULT_RECEPTION_IP_ADDRESS[] = "0.0.0.0";

//! constants for scip command test
enum CONSTANT_FOR_VLP_COMMAND_TEST {

    //! receive timeout [usec]
    RECEIVE_TIMEOUT_USEC = 1000,

    //! receive circular buffer size
    RECEIVE_CIRCULAR_BUFFER_SIZE = 10240,

    //! number of receives
    NUMBER_OF_RECEIVES = 100000,

    //! wait timeout to receive echoes
    WAIT_TIMEOUT_TO_RECEIVE_ECHOES = 48 * 1000 * 1000,

};


static void get_ip_address_and_port_number(const std::string &parameter_file_name,
                                           std::string &ip_address, std::string &port_number)
{

    ip_address = std::string(DEFAULT_SENSOR_IP_ADDRESS);
    port_number = std::string(DEFAULT_SENSOR_PORT_NUMBER);

    return;
}

static bool open_socket_for_vlp16_handler(int argc, char **argv,
                                          unsigned int communication_timeout_usec,
                                          vlp16_handler_t *handler)
{
    std::string parameter_file_name;
    if (argc > 1) {
        parameter_file_name = std::string(argv[1]);
    } else {
        parameter_file_name = std::string(DEFAULT_PARAMETER_FILE_NAME);
    }

    std::string ip_address;
    std::string port_number;

    get_ip_address_and_port_number(parameter_file_name,
                                   ip_address, port_number);

    if (open_socket_for_vlp16_handler(ip_address.c_str(), port_number.c_str(),
                                      DEFAULT_RECEPTION_IP_ADDRESS, port_number.c_str(),
                                      communication_timeout_usec,
                                      handler) == false) {
        cout << "fails.\n";
        release_winsock2_dynamic_link_library();
        return false;
    }
    cout << "success.\n";

    return true;
}
static void close_socket_and_release_memory_of_vlp16_handler(vlp16_handler_t *handler)
{

    //! todo stop data stream

    if (release_circular_buffer_of_vlp16_handler(handler) == false) {
        cout << "Release buffer failed.\n";
    }

    close_socket_of_vlp16_handler(handler);
    release_winsock2_dynamic_link_library();

    return;
}


int main(int argc, char **argv)
{
    vlp16_handler_t sensor;
    clear_vlp16_handler(&sensor);

    cout << "Open socket for VLP-16 ";
    if (open_socket_for_vlp16_handler(argc, argv,
                                      RECEIVE_TIMEOUT_USEC,
                                      &sensor) == false) {
        return 1;
    }

    cout << "Allocate memory ";
    if (allocate_circular_buffer_for_vlp16_handler(&sensor, VLP16_PACKET_VLP16,
                                                   RECEIVE_CIRCULAR_BUFFER_SIZE) == false) {

        cout << "failed.\n";
        close_socket_and_release_memory_of_vlp16_handler(&sensor);
    }
    cout << "success.\n";

    const double frontal_angle = 0.0 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;
    const double frontal_angle_torelance = 0.8 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

    const double elevation_angle_torelance = 0.5 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

    const double lowest_elevation_angle = -15.0 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

    lidar_echo_single_region_t lowest_frontal_region;
    set_lidar_echo_single_region_using_center_direction(frontal_angle, frontal_angle_torelance,
                                                        lowest_elevation_angle, elevation_angle_torelance,
                                                        &lowest_frontal_region);

    const double middle_elevation_angle = 1.0 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;
    lidar_echo_single_region_t middle_frontal_region;
    set_lidar_echo_single_region_using_center_direction(frontal_angle, frontal_angle_torelance,
                                                        middle_elevation_angle, elevation_angle_torelance,
                                                        &middle_frontal_region);

    const double highest_elevation_angle = 15.0 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;
    lidar_echo_single_region_t highest_frontal_region;
    set_lidar_echo_single_region_using_center_direction(frontal_angle, frontal_angle_torelance,
                                                        highest_elevation_angle, elevation_angle_torelance,
                                                        &highest_frontal_region);

    std::vector< lidar_echo_single_region_t > interest_regions;
    interest_regions.reserve(1 + 1 + 1);
    interest_regions.push_back(lowest_frontal_region);
    interest_regions.push_back(middle_frontal_region);
    interest_regions.push_back(highest_frontal_region);

    std::vector< std::vector<lidar_echo_data_t> > interest_echo_table(interest_regions.size());

    int received_data_byte = 0;
    unsigned int receive_count = 0;
    while (1) {

        if (receive_count > NUMBER_OF_RECEIVES) {
            cout << NUMBER_OF_RECEIVES << " packets are received.\n";
            break;
        }

        if (does_no_reply_after_long_waiting_occur(&sensor) == true) {
            cout << "No reply timeout.\n";
            break;
        }

        const bool packet_received =
            receive_vlp16_packet(&sensor, &received_data_byte,
                                 VLP16_PACKET_LENGTH);

        if (packet_received == true) {

            unsigned int captured_size = decode_vlp16_packet(&sensor);

            if (captured_size == 0) {
                cout << "Decode failed.\n";
                continue;
            }
            ++receive_count;
            std::vector<const lidar_line_data_t *> captured_lines;
            get_pointers_of_latest_unused_lidar_line_data(&sensor.line_data_buffer,
                                                          captured_size, captured_lines);

            std::vector< std::vector<lidar_echo_data_t> > temporal_interest_echoes(interest_regions.size());

            add_lidar_echo_data_in_each_single_region(captured_lines,
                                                      interest_regions,
                                                      temporal_interest_echoes);

            for (unsigned int i = 0; i < temporal_interest_echoes.at(1).size(); ++i) {

                output_lidar_echo_data_to_stream(cout, ',',
                                                 true, &temporal_interest_echoes.at(1).at(i));
                cout << "\n";
            }


            add_lidar_echo_data_in_each_single_region(captured_lines,
                                                      interest_regions, interest_echo_table);

        }

    }

    cout << "Number of interest echoes in middle region " <<  interest_echo_table.at(1).size() << "\n";
    for (unsigned int i = 0; i < interest_echo_table.at(1).size(); ++i) {
        /*
        output_lidar_echo_data_to_stream(cerr, ',', true,
                                         &interest_echo_table.at(1).at(i));
        cerr << "\n";
        */
    }

    unsigned int maximum_number_of_echoes = 200;

    std::vector<lidar_echo_data_t> highest_region_echoes;

    if (wait_to_receive_vlp16_measured_echoes(&sensor, WAIT_TIMEOUT_TO_RECEIVE_ECHOES,
                                              maximum_number_of_echoes,
                                              &highest_frontal_region,
                                              highest_region_echoes) == false) {
        cout << "Capturing failed.\n";
    } else {
        cout << "Capturing success.\n";

        cout << "Calculate statistics ";
        lidar_echo_statistics_t statistics;

        const double one_step_angle_value = 0.01 * LIDAR_DATA_ANGLE_COEFFICIENT_TO_CONVERT_DEGREE_TO_RADIAN;

        const double one_step_distance_value = 2.0;
        const double one_step_intensity_value = 2.0;

        const int small_bin_threshold = 0;

        calculate_statistics_of_lidar_echoes(highest_region_echoes,
                                             one_step_angle_value,
                                             one_step_distance_value, one_step_intensity_value,
                                             small_bin_threshold,
                                             (highest_frontal_region.minimum_horizontal_angle +
                                              highest_frontal_region.maximum_horizontal_angle) * 0.5,
                                             (highest_frontal_region.minimum_elevation_angle +
                                              highest_frontal_region.maximum_elevation_angle) * 0.5,
                                             statistics);


        cout << "end\n";

        output_lidar_echo_statistics_to_stream(cout, ',', true,
                                               &statistics);
        cout << "\n";
        output_lidar_echo_histogram_to_stream(cerr, ',', '\n',
                                              true, &statistics);
    }

    cout << "Close and release memory ";
    close_socket_and_release_memory_of_vlp16_handler(&sensor);
    cout << "end.\n";


    return 0;
}
