#ifndef VLP16_CONTROL_H
#define VLP16_CONTROL_H
/*!
  \file
  \brief functions to handle VLP16
  \author Kiyoshi MATSUO
  $Id$
*/

#include "socket_clientCtrl.h"

#include "lidar_dataCtrl.h"

//! length constants of vlp packet
enum LENGTH_CONSTANTS_OF_VLP16_PACKET {

    //! length of one data block
    VLP16_PACKET_DATA_BLOCK_LENGTH = 100,

    //! length of measured data on one firing sequence
    VLP16_PACKET_ONE_FIRING_SEQUENCE_DATA_LENGTH = 48,

    //! number of data block in one packet
    VLP16_PACKET_NUMBER_OF_DATA_BLOCKS = 12,

    //! half number of data block in one packet
    VLP16_PACKET_HALF_NUMBER_OF_DATA_BLOCKS = 6,

    //! length of header flag of data block
    VLP16_PACKET_HEADER_FLAG_OF_DATA_BLOCK_LENGTH = 2,

    //! length of azimuthal angle of data block
    VLP16_PACKET_AZIMUTHAL_ANGLE_LENGTH = 2,

    //! length of timestamp
    VLP16_PACKET_TIMESTAMP_LENGTH = 4,

    //! length of factory bit
    VLP16_PACKET_FACTORY_BYTE_LENGTH = 2,

    //! length of distance
    VLP16_PACKET_DISTANCE_LENGTH = 2,

    //! length of reflectivity
    VLP16_PACKET_REFLECTIVITY_LENGTH = 1,

    //! length of one packet (ethernet header byte [42byte] is excluded)
    VLP16_PACKET_LENGTH = 1206,

};

//! positions in VLP16 packet
enum POSITIONS_OF_VLP16_PACKET {

    //! timestamp position
    VLP16_PACKET_TIMESTAMP_POSITION = 1200,
    //! return mode byte position
    VLP16_PACKET_RETURN_MODE_BYTE_POSITION = 1204,
    //! sensor model byte position
    VLP16_PACKET_SENSOR_MODEL_BYTE_POSITION = 1205,

    //! flag position in data block
    VLP16_PACKET_HEADER_FLAG_POSITION_IN_DATA_BLOCK = 0,

    //! azimuthal angle position in data block
    VLP16_PACKET_AZIMUTHAL_ANGLE_POSITION_IN_DATA_BLOCK = 2,

    //! data position of odd firing sequence in data block
    VLP16_PACKET_ODD_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK = 4,

    //! data position of even firing sequence in data block
    VLP16_PACKET_EVEN_FIRING_SEQUENCE_POSITION_IN_DATA_BLOCK = 52,

};

//! data packet position array
const unsigned int VLP16_PACKET_DATA_BLOCK_POSITION[VLP16_PACKET_NUMBER_OF_DATA_BLOCKS] =
    { 0, 100, 200, 300, 400, 500,
      600, 700, 800, 900, 1000, 1100 };

//! constants for VLP16 return mode
enum VLP16_PACKET_RETURN_MODE {
    //! invalid return mode
    VLP16_PACKET_INVALID_RETURN_MODE = -1,

    //! strongest return mode
    VLP16_PACKET_STRONGEST_RETURN_MODE = 0,

    //! last return mode
    VLP16_PACKET_LAST_RETURN_MODE,

    //! dual return mode
    VLP16_PACKET_DUAL_RETURN_MODE,

    //! number of return modes
    NUMBER_OF_RETURN_MODES_IN_VLP16_PACKET,
};

//! byte value table of return modes in VLP16 packet
const char VLP16_PACKET_RETURN_MODE_BYTE[NUMBER_OF_RETURN_MODES_IN_VLP16_PACKET] =
    { 0x37, 0x38, 0x39};

//! consts for sensor model
enum VLP16_PACKET_SENSOR_MODEL {
    //! invalid sensor model
    VLP16_PACKET_INVALID_SENSOR_MODEL = -1,

    //! HDL-32E
    VLP16_PACKET_HDL_32E = 0,
    //! VLP-16
    VLP16_PACKET_VLP16,

    //! number of sensor models
    NUMBER_OF_SENSOR_MODELS_IN_VLP16_PACKET,
};

//! byte value table of sensor model in VLP16 packet
const char VLP16_PACKET_SENSOR_MODEL_BYTE[NUMBER_OF_SENSOR_MODELS_IN_VLP16_PACKET] =
    { 0x21, 0x22 };

//! number of spots of each sensor model
const unsigned int VLP16_PACKET_NUMBER_OF_SPOTS[NUMBER_OF_SENSOR_MODELS_IN_VLP16_PACKET] =
    { 32, 16 };

//! flag bytes of data block in VLP16 packet
const char VLP16_PACKET_HEADER_FLAG_OF_DATA_BLOCK[VLP16_PACKET_HEADER_FLAG_OF_DATA_BLOCK_LENGTH] =
    { (char)0xFF, (char)0xEE };

//! structure for evaluate communication status
struct vlp16_communication_status_t {

    //! buffer size error
    bool buffer_error_occurs;

    //! flag to check decode error
    bool decode_error_occurs;

    //! flag for memory allocation
    bool memory_allocated;

    //! flag for socket
    bool socket_opened;

};

//! azimuthal angle scale factor of VLP16 packet [rad] 0.01 * PI / 180.0
#define VLP16_PACKET_AZIMUTHAL_ANGLE_SCALE 0.00017453292

//! distance scale factor of VLP16 [mm]
#define VLP16_PACKET_DISTANCE_SCALE 2.0

//! one laser firing interval (usec)
#define VLP16_PACKET_ONE_LASER_FIRING_INTERVAL_USEC 2.304

//! laser recharge cycle (usec)
#define VLP16_PACKET_LASER_RECHARGE_CYCLE_USEC 18.432

//! one firing sequence interval (usec) 2.304 * 16 + 18.432
#define VLP16_PACKET_LASER_FIRING_SEQUENCE_USEC 55.296

//! maximum azimuthal angle difference of one data block 7200.0 * 0.001 * 0.001 * 55.296 * 2 [0.01 degree]
#define VLP16_PACKET_MAXIMUM_AZIMUTHAL_ANGLE_OF_ONE_DATA_BLOCK_DEGREE 79.62624

//! one data block (two firing sequences) interval (usec) (2.304 * 16 + 18.432) * 2
#define VLP16_PACKET_ONE_DATA_BLOCK_USEC 110.592

//! one firing ration to one data block 2.304 / 110.592
#define VLP16_PACKET_ONE_LASER_FIRING_RATIO_TO_ONE_FIRING_SEQUENCE 0.020833333

//! elevation angle tables of VLP-16 (specification value) [degree]
const double VLP16_SPOT_SPECIFIVATION_ELEVATION_ANGLE_DEGREE_ARRAY[] =
    {-15.0, 1.0,
     -13.0, 3.0,
     -11.0, 5.0,
     -9.0, 7.0,
     -7.0, 9.0,
     -5.0, 11.0,
     -3.0, 13.0,
     -1.0, 15.0};

//! constants of VLP16 packet
enum VLP16_PACKET_CONSTANTS {
    //! maximum azimuthal angle
    VLP16_PACKET_MAXIMUM_AZIMUTHAL_ANGLE = 36000,

    //! maximum timestamp
    VLP16_PACKET_MAXIMUM_TIMESTAMP = 3600000000U,

    //! minimum rotation speed [degree/second]
    VLP16_PACKET_MINIMUM_ROTATION_SPEED = 1800,

    //! maximum rotation speed [degree/second]
    VLP16_PACKET_MAXIMUM_ROTATION_SPEED = 7200,

    //! azimuthal angle threshold of continuous data blocks [0.01 degree]
    VLP16_PACKET_AZIMUTHAL_ANGLE_THRESHOLD_OF_CONTINUOUS_DATA_BLOCKS = 160

};

//! constants for communication handler
enum VLP16_HANDER_CONSTANTS {

    //! number of store buffer of data block
    NUMBER_OF_STORING_VLP16_PACKET_DATA_BLOCKS = 11,

    //! maximum interval of reply [usec]
    VLP16_COMMUNICATION_HANDLER_MAXIMUM_NO_REPLY_INTERVAL_ON_AWAITING_PACKETS_USEC = 5 * 1000 * 1000,

    //! number of lines to store measured data
    NUMBER_OF_VLP16_PACKET_LINES_TO_STORE_MEASURED_DATA = 32,
};

//! communication handler
struct vlp16_handler_t {

    //! timer to evaluate communication process
    TimeTheInterval timer;

    //! timer to check time of no-reply
    TimeTheInterval no_reply_interval_timer;

    //! handler of socket communication
    socket_client_t socket_handler;

    //! flags of communication status
    vlp16_communication_status_t communication_status;

    //! communication timeout [usec]
    int communication_timeout_usec;

    //! buffer to decode one packet
    char decode_buffer[VLP16_PACKET_LENGTH];

    //! data block accessor
    const char *decoding_data_blocks[VLP16_PACKET_NUMBER_OF_DATA_BLOCKS];

    //! timestamp of current decoding packet
    unsigned int decoding_packet_timestamp_usec;

    //! return mode of current decoding packet
    enum VLP16_PACKET_RETURN_MODE decoding_packet_return_mode;

    //! sensor model of current decoding packet
    enum VLP16_PACKET_SENSOR_MODEL decoding_packet_sensor_model;

    //! return mode of past decoded packet
    enum VLP16_PACKET_RETURN_MODE past_packet_return_mode;
    //! sensor model of past decoded packet
    enum VLP16_PACKET_SENSOR_MODEL past_packet_sensor_model;

    //! timestamp of past decoded packet
    unsigned int past_packet_timestamp_usec;

    //! elevation angle table
    std::vector<double> elevation_angle_array;
    //! minimum elevation angle
    double minimum_elevation_angle;
    //! maximum elevation angle
    double maximum_elevation_angle;

    //! buffer to store last data block, in which data on even firing sequence do not have azimuthal angle.
    char remaining_data_block_buffer[NUMBER_OF_STORING_VLP16_PACKET_DATA_BLOCKS][VLP16_PACKET_DATA_BLOCK_LENGTH];
    //! number of current remaining data blocks
    unsigned int number_of_remaining_data_blocks;

    //! circular buffer for measured line data
    lidar_line_circular_buffer_t line_data_buffer;
};

/*!
  \brief function to clear vlp16 handler
*/
extern void clear_vlp16_handler(vlp16_handler_t *handler);

/*!
  \brief function to allocate circular buffer for communication with VLP16
  \attention if buffer_length_byte < VLP16_PACKET_LENGTH, this function does not work and return false as invalid buffer size.
*/
extern bool allocate_circular_buffer_for_vlp16_handler(vlp16_handler_t *vlp16_handler,
                                                       enum VLP16_PACKET_SENSOR_MODEL sensor_model,
                                                       unsigned int buffer_length_byte);

/*!
  \brief function to release circular buffer of vlp16 communication handler
  \attention this function does not work if memory_allocated == false
*/
extern bool release_circular_buffer_of_vlp16_handler(vlp16_handler_t *vlp16_handler);

/*!
  \brief function to open socket for communication with VLP16
  \attention this function does not work if socket_opened == true.
  \attention this function clears decode buffer.
*/
extern bool open_socket_for_vlp16_handler(const char *destination_ip_address_string,
                                          const char *destination_port_number_string,
                                          const char *reception_ip_address_string,
                                          const char *reception_port_number_string,
                                          int communication_timeout_usec,
                                          vlp16_handler_t *vlp16_handler);

/*!
  \brief function to close socket of vlp16 communication handler
*/
extern void close_socket_of_vlp16_handler(vlp16_handler_t *vlp16_handler);

/*!
  \brief function to erase all receiving data and decoding data in buffer
*/
extern void erase_all_receiving_and_decoding_data(vlp16_handler_t *handler);

/*!
  \brief function to receive vlp16 packet
  \attention this function returns true if one vlp16 packet is received
  \attention this function does not decode packet
  \attention this function evaluate validations of all header flags
  \attention this function renew decoding_packet_timestamp_usec, decoding_packet_return_mode, and decoding_packet_sensor_model
*/
extern bool receive_vlp16_packet(vlp16_handler_t *vlp16_handler, int *received_data_length,
                                 unsigned int onetime_receive_length_byte);

/*!
  \brief function to decode received vlp16 packet
*/
extern unsigned int decode_vlp16_packet(vlp16_handler_t *vlp16_handler);

/*!
  \brief function to check no reply interval
  \attention this function return true if it spends VLP16_COMMUNICATION_HANDLER_MAXIMUM_NO_REPLY_INTERVAL_ON_AWAITING_PACKETS_USEC after last receiving
*/
extern bool does_no_reply_after_long_waiting_occur(const vlp16_handler_t *vlp16_handler);

/*!
  \brief function to wait to receive echoes
*/
extern bool wait_to_receive_vlp16_measured_echoes(vlp16_handler_t *vlp16_handler,
                                                  int wait_timeout_usec,
                                                  unsigned int maximum_number_of_echoes,
                                                  const lidar_echo_single_region_t *region,
                                                  std::vector<lidar_echo_data_t> &echoes);

#endif // VLP16_CONTROL_H
