#ifndef SOCKET_CLIENT_CONTROL_H
#define SOCKET_CLIENT_CONTROL_H
/*!
  \file
  \brief functions to handle socket client
  \author Kiyoshi MATSUO
  $Id$
*/

#include "environmentCtrl.h"

#include "byte_arrayCtrl.h"

#include "timeCtrl.h"

//! constants for socket communication
enum SOCKET_CLIENT_CONSTANT {
    //! length of ip address
    SOCKET_CLIENT_DESTINATION_IP_ADDRESS_STRING_LENGTH = 16,
    //! length of port name
    SOCKET_CLIENT_DESTINATION_PORT_STRING_LENGTH = 8,
    //! receive buffer size
    SOCKET_CLIENT_RECEIVE_BUFFER_LENGTH = 4096,

    //! connection timeout [sec]
    SOCKET_CLIENT_CONNECTION_TIMEOUT_SEC = 2,

    //! invalid file descriptor of tcp client
    SOCKET_CLIENT_INVALID_FILE_DESCRIPTOR = -1,

    //! invalid return value of receiving message
    SOCKET_CLIENT_INVALID_RETURN_VALUE = -1,

    //! invalid socket type
    SOCKET_CLIENT_INVALID_SOCKET_TYPE = -1,

    //! length of protocol names
    LENGTH_OF_SOCKET_USABLE_PROTOCOL_NAME = 3,

    //! length of exceptional ip address of inet_addr
    LENGTH_OF_EXCEPTIONAL_IP_ADDRESS_STRING = 15,
};

//! exceptional ip address of inet_addr
const char EXCEPTIONAL_IP_ARRESS_STRING[LENGTH_OF_EXCEPTIONAL_IP_ADDRESS_STRING + 1] =
    "255.255.255.255";

//! enumeration of usable protocols
enum SOCKET_PROTOCOL {

    //! invalid protocol type
    INVALID_SOCKET_PROTOCOL = -1,

    //! tcp
    SOCKET_PROTOCOL_TCP = 0,

    //! udp
    SOCKET_PROTOCOL_UDP,

    //! number of usable protocols
    NUMBER_OF_SOCKET_USABLE_PROTOCOLS,

};

//! table of usable protocol names
const char SOCKET_PROTOCOL_NAME[NUMBER_OF_SOCKET_USABLE_PROTOCOLS][LENGTH_OF_SOCKET_USABLE_PROTOCOL_NAME + 1] =
    { "tcp", "udp" };

//! structure for socket client
struct socket_client_t {

    //! protocol type
    enum SOCKET_PROTOCOL protocol;

    //! ip address of destination server
    char destination_ip_address[SOCKET_CLIENT_DESTINATION_IP_ADDRESS_STRING_LENGTH];

    //! port number of destination server
    char destination_port_number[SOCKET_CLIENT_DESTINATION_PORT_STRING_LENGTH];

    //! ip address for reception
    char reception_ip_address[SOCKET_CLIENT_DESTINATION_PORT_STRING_LENGTH];

    //! port number of reception
    char reception_port_number[SOCKET_CLIENT_DESTINATION_PORT_STRING_LENGTH];

    //! past blocking mode flag
    bool past_blocking_mode_flag;

    //! file descriptor of client
    int file_descriptor;

    //! file descriptor of client (for send on UDP packet with receiving multicasted UDP packet)
    int send_file_descriptor;

    //! receive circular buffer
    circular_buffer_t buffer;

};

/*!
  \brief function to clear receive buffer of socket client structure
*/
extern bool allocate_circular_receive_buffer_of_socket_client(socket_client_t *client,
                                                              unsigned int buffer_length_byte);

/*!
 \brief function to release receive buffer of socket client structure
*/
extern bool release_circular_receive_buffer_of_socket_client(socket_client_t *client);

/*!
  \brief function to set ip address and port on socket client structure
*/
extern bool set_ip_address_and_port_on_socket_client(const char *ip_address_string,
                                                     const char *port_number_string,
                                                     socket_client_t * client);

/*!
  \brief function to output destination ip address and port number of socket client structure
*/
extern void output_destination_ip_address_and_port_number_of_socket_client(std::ostream &stream,
                                                                           const socket_client_t *client);

/*!
  \brief function to open socket as client
  \attention this function uses connect even if protocol type is UDP. (This client can communicate with only one server)
*/
extern bool open_socket_for_client(socket_client_t *client);

/*!
  \brief function to open socket for client
*/
extern bool open_socket_for_client(const char *ip_address_string, const char *port_number_string,
                                   const char *reception_ip_address_string,
                                   const char *reception_port_number_string,
                                   enum SOCKET_PROTOCOL protocol,
                                   socket_client_t *client);

/*!
  \brief function to close socket
*/
extern void close_socket_of_client(socket_client_t *client);

/*!
  \brief function to release dll of winsock2
  \attention this function is defined for only windows
*/
extern void release_winsock2_dynamic_link_library(void);

/*!
  \brief function to get readable byte size from tcp client
*/
extern int query_readable_byte_size(socket_client_t *client);

/*!
  \brief function to receive data
*/
extern int receive_data_using_socket_client(socket_client_t *client,
                                            char *receive_buffer,
                                            unsigned int receive_buffer_size,
                                            int timeout_usec);

/*!
  \brief function to send data
*/
extern int send_data_using_socket_client(socket_client_t *client,
                                         const char *send_buffer, unsigned int send_buffer_size,
                                         int timeout_usec);

/*!
  \brief function to receive constant length message
*/
extern int receive_constant_length_data(socket_client_t *client, int receive_timeout_usec,
                                        unsigned int data_length_byte, char *destination_buffer,
                                        unsigned int onetime_receive_length_byte,
                                        unsigned int *captured_data_length, unsigned int *destination_copied_data_length);

/*!
  \brief function to receive code tailed message
*/
extern int receive_code_tailed_message(socket_client_t *client, int receive_timeout_usec,
                                       const char *code, unsigned int code_length,
                                       char *destination_buffer, unsigned int destination_buffer_length,
                                       unsigned int *captured_data_length, unsigned int *destination_copied_data_length,
                                       unsigned int *tailed_message_length);

//! parameters to handle udp communication
enum UDP_COMMUNICATION_PARAMETERS {
    //! invalid parameter of udp communication
    INVALID_UDP_COMMUNICATION_PARAMETER = -1,

    //! udp communication configure
    UDP_COMMUNICATION_CONFIGURATION_PARAMETER,

    //! number of setting parameters
    NUMBER_OF_UDP_COMMUNICATION_PARAMETERS,

    //! maximum length of tcp/ip communication parameter tag
    MAXIMUM_LENGTH_OF_UDP_COMMUNICATION_PARAMETER_TAG = 32,
};

//! parameter tag table for udp communication
const char UDP_COMMUNICATION_PARAMETER_TAG_TABLE[NUMBER_OF_UDP_COMMUNICATION_PARAMETERS][MAXIMUM_LENGTH_OF_UDP_COMMUNICATION_PARAMETER_TAG] =
    {"udp_configure"};

const unsigned int UDP_COMMUNICATION_PARAMETER_SIZE_TABLE[NUMBER_OF_UDP_COMMUNICATION_PARAMETERS] =
    {6};

/*!
  \brief function to decode string table to udp parameter
*/
extern void decode_string_table_to_udp_communication_parameter(const std::vector< std::vector<std::string> > &string_table,
                                                               std::string &ip_address, std::string &port_number,
                                                               std::string &reception_ip_address_string,
                                                               std::string &reception_port_number_string,
                                                               int *communication_timeout_usec,
                                                               std::string &protocol_string);



#endif // SOCKET_CLIENT_CONTROL_H
