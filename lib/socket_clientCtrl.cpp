
#include "socket_clientCtrl.h"

// for strlen strcpy
#include <string.h>

// include for atoi
#include <stdlib.h>

#if defined (WINDOWS_OS)
#include <WinSock2.h>
#else

// for sockaddr_in getservbyport
#include <netdb.h>
// for inet_addr
#include <arpa/inet.h>

// for close
#include <unistd.h>

// for ioctl
#include <sys/ioctl.h>

#endif

// for errno
#include <errno.h>

// for fcntl
#include <fcntl.h>

bool allocate_circular_receive_buffer_of_socket_client(socket_client_t *client,
                                                       unsigned int buffer_length_byte)
{

    initialize_circular_buffer(&client->buffer);

    return allocate_memory_for_circular_buffer(&client->buffer, buffer_length_byte);
}

bool release_circular_receive_buffer_of_socket_client(socket_client_t *client)
{
    return release_memory_of_circular_buffer(&client->buffer);
}

bool set_ip_address_and_port_on_socket_client(const char *ip_address_string,
                                              const char *port_number_string,
                                              socket_client_t * client)
{

    if ((strlen(ip_address_string) > SOCKET_CLIENT_DESTINATION_IP_ADDRESS_STRING_LENGTH) ||
        (strlen(port_number_string) > SOCKET_CLIENT_DESTINATION_PORT_STRING_LENGTH)) {

        return false;

    }

    strcpy(client->destination_ip_address, ip_address_string);
    strcpy(client->destination_port_number, port_number_string);

    return true;
}

bool set_reception_ip_address_and_port_on_socket_client(const char *ip_address_string,
                                                        const char *port_number_string,
                                                        socket_client_t *client)
{

    if ((strlen(ip_address_string) > SOCKET_CLIENT_DESTINATION_IP_ADDRESS_STRING_LENGTH) ||
        (strlen(port_number_string) > SOCKET_CLIENT_DESTINATION_PORT_STRING_LENGTH)) {

        return false;

    }

    strcpy(client->reception_ip_address, ip_address_string);
    strcpy(client->reception_port_number, port_number_string);

    return true;

}

static bool set_protocol_on_socket_client(socket_client_t *client,
                                          enum SOCKET_PROTOCOL protocol)
{
    if (protocol < 0) {
        return false;
    }
    client->protocol = protocol;

    return true;
}

void output_destination_ip_address_and_port_number_of_socket_client(std::ostream &stream,
                                                                    const socket_client_t *client)
{

    stream << client->destination_ip_address << ":"
           << client->destination_port_number;

    return;
}

static bool set_port_number_to_connect_socket_server(struct sockaddr_in *destination,
                                                     const char *port_number_string, enum SOCKET_PROTOCOL protocol)
{
    if ((protocol < 0) ||
        (protocol >= NUMBER_OF_SOCKET_USABLE_PROTOCOLS)) {
        return false;
    }

    int port_number = -1;
    port_number = atoi(port_number_string);

    if (port_number <= 0) {

        // seek service
        struct servent *ent = NULL;
        ent = getservbyname(port_number_string,
                            SOCKET_PROTOCOL_NAME[protocol]);

        if (ent == NULL) {
            return false;
        }
        destination->sin_port = ent->s_port;

    } else {

        destination->sin_port = htons(port_number);

    }

    return true;
}

static bool verify_connection_to_socket_server(int return_of_connect, int error_number,
                                               socket_client_t *client, int retry_timeout_sec)
{

    if (return_of_connect == 0) {
        return true;
    }

#if defined(WINDOWS_OS)
    if (return_of_connect != SOCKET_ERROR) {
        return false;
    }
    int windows_error_number = WSAGetLastError();

    if (windows_error_number != WSAEWOULDBLOCK) {
        return false;
    }

#else
    if (error_number != EINPROGRESS) {
        return false;
    }
#endif

    fd_set write_file_descriptors;

    FD_ZERO(&write_file_descriptors);
    FD_SET(client->send_file_descriptor, &write_file_descriptors);

    struct timeval time_value;
    time_value.tv_sec = retry_timeout_sec;
    time_value.tv_usec = 0;

    const int return_of_select =
        select(client->send_file_descriptor + 1,
			   NULL,
               &write_file_descriptors,
               NULL, &time_value);

    if (return_of_select <= 0) {
        return false;
    }

    int socket_option_value = -1;
    int socket_option_value_size = sizeof(socket_option_value);

#if defined(WINDOWS_OS)
    if (getsockopt(client->send_file_descriptor, SOL_SOCKET, SO_ERROR,
                   (char *)&socket_option_value, &socket_option_value_size) != 0) {
        return false;
    }
#else
    if (getsockopt(client->send_file_descriptor, SOL_SOCKET, SO_ERROR,
                   &socket_option_value, (socklen_t *)&socket_option_value_size) != 0) {
        return false;
    }
#endif

    if (socket_option_value != 0) {
        return false;
    }

    return true;
}

static bool set_block_mode_on_socket_client(socket_client_t *client, bool block_mode)
{
    //! this function only set receive descriptor (send_file_descriptor for UDP communication is not changed by this function)

    if (block_mode == true) {

#if defined (WINDOWS_OS)
		u_long descriptor_flag = 0;
		if (ioctlsocket(client->file_descriptor, FIONBIO, &descriptor_flag) != 0) {
            return false;
        }
#else
		int descriptor_flag = 0;
		descriptor_flag = fcntl(client->file_descriptor, F_GETFL, 0);
        if (descriptor_flag < 0) {
            return false;
        }

		if (fcntl(client->file_descriptor, F_SETFL, descriptor_flag & (~O_NONBLOCK)) < 0) {
            return false;
        }
#endif

        client->past_blocking_mode_flag = true;

    } else {

#if defined (WINDOWS_OS)
		u_long descriptor_flag = 1;
		if (ioctlsocket(client->file_descriptor, FIONBIO, &descriptor_flag) != 0) {
            return false;
        }
#else
		int descriptor_flag = 0;
		descriptor_flag = fcntl(client->file_descriptor, F_GETFL, 0);
        if (descriptor_flag < 0) {
            return false;
        }

		if (fcntl(client->file_descriptor, F_SETFL, descriptor_flag | O_NONBLOCK) < 0) {
            return false;
        }
#endif
        client->past_blocking_mode_flag = false;

    }

    return true;
}

static int get_socket_type_from_protocol(enum SOCKET_PROTOCOL protocol)
{
    int type = SOCKET_CLIENT_INVALID_SOCKET_TYPE;

    switch (protocol) {

        case SOCKET_PROTOCOL_TCP:
            type = SOCK_STREAM;
            break;

        case SOCKET_PROTOCOL_UDP :
            type = SOCK_DGRAM;
            break;

        default:
            break;
    }

    return type;
}

static bool get_in_addr_from_ip_address_string(const char* ip_address, in_addr *address)
{
    if (are_equivalent_byte_sequences(ip_address, EXCEPTIONAL_IP_ARRESS_STRING,
                                      LENGTH_OF_EXCEPTIONAL_IP_ADDRESS_STRING) == true) {

        address->s_addr = -1;
        return true;
    }

    address->s_addr = inet_addr(ip_address);

    if (address->s_addr == INADDR_NONE) {
        // get info of host as name
        struct hostent* host = NULL;
        host = gethostbyname(ip_address);

        if (host == NULL) {
            return false;
        }
        memcpy(address, (struct in_addr *) *host->h_addr_list,
               sizeof(struct in_addr));
    }

    return true;
}


bool open_socket_for_client(socket_client_t *client)
{

#if defined(WINDOWS_OS)
	// initialize winsock2
	WSADATA wsa_data;
	int return_of_wsa_startup = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (return_of_wsa_startup != 0) {
		return false;
	}
#endif

    struct sockaddr_in destination_server;

    // zero clear
    memset(&destination_server, 0, sizeof(destination_server));
    // set address family
    destination_server.sin_family = AF_INET;

    struct in_addr address;
    if (get_in_addr_from_ip_address_string(client->destination_ip_address,
                                           &address) == false) {
        return false;
    }

    // set destination ip address
    destination_server.sin_addr = address;

    // set destination port number
    if (set_port_number_to_connect_socket_server(&destination_server,
                                                 client->destination_port_number,
                                                 client->protocol) == false) {

        return false;
    }

    const int socket_type =
        get_socket_type_from_protocol(client->protocol);

    if (socket_type == SOCKET_CLIENT_INVALID_SOCKET_TYPE) {
        return false;
    }

    // generate socket
    client->file_descriptor = socket(AF_INET, socket_type, 0);
    if (client->file_descriptor < 0) {
        return false;
    }

#if defined(WINDOWS_OS)
	u_long descriptor_flag = 1;
	if (ioctlsocket(client->file_descriptor, FIONBIO, &descriptor_flag) != 0) {
        return false;
    }
#else
    int descriptor_flag = 0;
    descriptor_flag = fcntl(client->file_descriptor, F_GETFL, 0);
    fcntl(client->file_descriptor, F_SETFL, descriptor_flag | O_NONBLOCK);
#endif

    if (client->protocol == SOCKET_PROTOCOL_UDP) {

        struct sockaddr_in reception_address;
        reception_address.sin_family = AF_INET;

        struct in_addr address_value;
        // set reception ip address
        if (get_in_addr_from_ip_address_string(client->reception_ip_address,
                                               &address_value) == false) {
            return false;
        } else {
            reception_address.sin_addr = address_value;
        }

        // set reception port number
        if (set_port_number_to_connect_socket_server(&reception_address,
                                                     client->reception_port_number,
                                                     client->protocol) == false) {

            return false;
        }

#if defined(WINDOWS_OS)
        const int reuse_address_on = 1;
        setsockopt(client->file_descriptor,
                   SOL_SOCKET, SO_REUSEADDR,
                   (const char*)&reuse_address_on, sizeof(reuse_address_on));
#else
        const int reuse_address_on = 1;
        setsockopt(client->file_descriptor,
                   SOL_SOCKET, SO_REUSEADDR,
                   (const void*)&reuse_address_on, (socklen_t)sizeof(reuse_address_on));
#endif
        const int return_of_bind =
            bind(client->file_descriptor,
                 (struct sockaddr *)&reception_address, sizeof(reception_address));

        if (return_of_bind != 0) {
            return false;
        }

        client->send_file_descriptor = socket(AF_INET, socket_type, 0);
        if (client->send_file_descriptor < 0) {
            return false;
        }

#if defined(WINDOWS_OS)
        u_long descriptor_flag = 1;
        if (ioctlsocket(client->send_file_descriptor, FIONBIO, &descriptor_flag) != 0) {
            return false;
        }
#else
        int descriptor_flag = 0;
        descriptor_flag = fcntl(client->send_file_descriptor, F_GETFL, 0);
        fcntl(client->send_file_descriptor, F_SETFL, descriptor_flag | O_NONBLOCK);
#endif

    } else {
        client->send_file_descriptor = client->file_descriptor;
    }

    const int return_of_connect =
        connect(client->send_file_descriptor,
                (struct sockaddr *)&destination_server, sizeof(destination_server));

    if (return_of_connect == 0) {
        return true;
    }

    if(verify_connection_to_socket_server(return_of_connect, errno,
                                          client, SOCKET_CLIENT_CONNECTION_TIMEOUT_SEC) == false) {

        close_socket_of_client(client);
        return false;
    }

    set_block_mode_on_socket_client(client, false);

    return true;
}

bool open_socket_for_client(const char *ip_address_string,
                            const char *port_number_string,
                            const char *reception_ip_address_string,
                            const char *reception_port_number_string,
                            enum SOCKET_PROTOCOL protocol,
                            socket_client_t *client)
{
    if (set_ip_address_and_port_on_socket_client(ip_address_string, port_number_string,
                                                 client) == false) {
        return false;
    }

    if (set_protocol_on_socket_client(client, protocol) == false) {
        return false;
    }

    if (protocol == SOCKET_PROTOCOL_UDP) {

        if (set_reception_ip_address_and_port_on_socket_client(reception_ip_address_string,
                                                               reception_port_number_string,
                                                               client) == false) {
            return false;
        }

    }

    return open_socket_for_client(client);
}

void close_socket_of_client(socket_client_t *client)
{
    if (client->file_descriptor >= 0) {
#if defined(WINDOWS_OS)
		shutdown(client->file_descriptor, SD_BOTH);
        closesocket(client->file_descriptor);
#else
        close(client->file_descriptor);
#endif
        client->file_descriptor = SOCKET_CLIENT_INVALID_FILE_DESCRIPTOR;
    }
    return;
}

void release_winsock2_dynamic_link_library(void)
{
#if defined(WINDOWS_OS)
    //! one cleanup in one process
    WSACleanup();
#endif
    return;
}

int query_readable_byte_size(socket_client_t *client)
{
    int byte_size = SOCKET_CLIENT_INVALID_RETURN_VALUE;

#if defined(WINDOWS_OS)

    u_long readable_byte_size = 0;
    if (ioctlsocket(client->file_descriptor, FIONREAD, &readable_byte_size) != 0) {
        return byte_size;
    }

    byte_size = (int)readable_byte_size;

#else
    int readable_byte_size = 0;

    if (ioctl(client->file_descriptor, FIONREAD, &readable_byte_size) != 0) {
        return byte_size;
    }
    byte_size = (int)readable_byte_size;

#endif
    return byte_size;

}

int receive_data_using_socket_client(socket_client_t *client,
                                     char *receive_buffer, unsigned int receive_buffer_size,
                                     int timeout_usec)
{
    int received_size = -1;

    struct timeval timeout_value;
    fd_set receive_checking_file_descriptors;
    int return_of_select = 0;
    if (timeout_usec >= 0) {

        timeout_value.tv_sec = timeout_usec / 1000000;
        timeout_value.tv_usec = timeout_usec % 1000000;

        FD_ZERO(&receive_checking_file_descriptors);
        FD_SET(client->file_descriptor, &receive_checking_file_descriptors);

        return_of_select =
            select(client->file_descriptor + 1, &receive_checking_file_descriptors, NULL, NULL,
                   &timeout_value);

        if (return_of_select <= 0) {
            return received_size;
        }

        set_block_mode_on_socket_client(client, false);

    } else {

        set_block_mode_on_socket_client(client, true);

    }

    received_size = recv(client->file_descriptor, receive_buffer, receive_buffer_size, 0);

    return received_size;
}

int send_data_using_socket_client(socket_client_t *client,
                                  const char *send_buffer, unsigned int send_buffer_size,
                                  int timeout_usec)
{
    int sended_size = -1;

    struct timeval timeout_value;
    fd_set send_checking_file_descriptors;
    int return_of_select = 0;
    if (timeout_usec >= 0) {

        timeout_value.tv_sec = timeout_usec / 1000000;
        timeout_value.tv_usec = timeout_usec % 1000000;

        FD_ZERO(&send_checking_file_descriptors);
        FD_SET(client->send_file_descriptor, &send_checking_file_descriptors);

        return_of_select =
            select(client->send_file_descriptor + 1, NULL, &send_checking_file_descriptors, NULL,
                   &timeout_value);

        if (return_of_select <= 0) {
            return sended_size;
        }

        set_block_mode_on_socket_client(client, false);

    } else {

        set_block_mode_on_socket_client(client, true);

    }

    sended_size = send(client->send_file_descriptor, send_buffer, send_buffer_size, 0);

    return sended_size;
}

int receive_constant_length_data(socket_client_t *client, int receive_timeout_usec,
                                 unsigned int data_length_byte, char *destination_buffer,
                                 unsigned int onetime_receive_length_byte,
                                 unsigned int *captured_data_length, unsigned int *destination_copied_data_length)
{
    int received_size = 0;

    *captured_data_length = 0;
    *destination_copied_data_length = 0;

    if (client->buffer.length_byte < data_length_byte) {
        return SOCKET_CLIENT_INVALID_RETURN_VALUE;
    }

    unsigned int remaining_data_length =
        calculate_remaining_data_length(&client->buffer);

    if (remaining_data_length >= data_length_byte) {

        *destination_copied_data_length =
            copy_non_used_data_byte_from_circular_buffer(&client->buffer, data_length_byte,
                                                         destination_buffer, data_length_byte);
        return received_size;
    }

    unsigned int receive_length_byte = onetime_receive_length_byte;

    if (receive_length_byte > client->buffer.empty_buffer_length_byte) {
        receive_length_byte = client->buffer.empty_buffer_length_byte;
    }

    char *temporal_buffer = NULL;

    temporal_buffer = (char *)malloc(receive_length_byte * sizeof(char));

    if (temporal_buffer == NULL) {
        return SOCKET_CLIENT_INVALID_RETURN_VALUE;
    }

    received_size =
        receive_data_using_socket_client(client, temporal_buffer,
                                         receive_length_byte,
                                         receive_timeout_usec);

    if (received_size > 0) {

        *captured_data_length =
            copy_byte_to_circular_buffer(temporal_buffer, (unsigned int)received_size,
                                         &client->buffer);

    }

    free(temporal_buffer);

    remaining_data_length =
        calculate_remaining_data_length(&client->buffer);

    if (remaining_data_length >= data_length_byte) {

        *destination_copied_data_length =
            copy_non_used_data_byte_from_circular_buffer(&client->buffer, data_length_byte,
                                                         destination_buffer, data_length_byte);
        return received_size;
    }

    return received_size;
}


int receive_code_tailed_message(socket_client_t *client, int receive_timeout_usec,
                                const char *code, unsigned int code_length,
                                char *destination_buffer, unsigned int destination_buffer_length,
                                unsigned int *captured_data_length, unsigned int *destination_copied_data_length,
                                unsigned int *tailed_message_length)
{
    int received_size = 0;

    *captured_data_length = 0;
    *destination_copied_data_length = 0;

    const unsigned int empty_buffer_length =
        get_length_of_empty_data_buffer(&client->buffer);

    char *temporal_buffer = NULL;

    temporal_buffer = (char *)malloc(empty_buffer_length * sizeof(char));

    if (temporal_buffer == NULL) {
        return SOCKET_CLIENT_INVALID_RETURN_VALUE;
    }

    int actual_receive_timeout_usec = receive_timeout_usec;
    const unsigned int remaining_data_length =
        calculate_remaining_data_length(&client->buffer);

    if (remaining_data_length >= code_length) {
        actual_receive_timeout_usec = 0;
    }

    received_size =
        receive_data_using_socket_client(client, temporal_buffer,
                                         empty_buffer_length,
                                         actual_receive_timeout_usec);

    if (received_size > 0) {

        *captured_data_length =
            copy_byte_to_circular_buffer(temporal_buffer, (unsigned int)received_size,
                                         &client->buffer);

    }

    free(temporal_buffer);

    *destination_copied_data_length =
        copy_first_tailed_byte_array_from_circular_buffer(&client->buffer,
                                                          code, code_length,
                                                          destination_buffer, destination_buffer_length,
                                                          tailed_message_length);

    return received_size;
}

static void parse_string_to_udp_communication_parameter(const std::vector<std::string> &string_array,
                                                        std::string &ip_address, std::string &port_number,
                                                        std::string &reception_ip_address_string,
                                                        std::string &reception_port_number_string,
                                                        int *communication_timeout_usec,
                                                        std::string &protocol_string)
{
    if (string_array.size() == 0) {
        return;
    }

    int parameter_index = 0;
    for (parameter_index = 0; parameter_index < NUMBER_OF_UDP_COMMUNICATION_PARAMETERS; ++parameter_index) {

        if (strcmp(string_array.at(0).c_str(), UDP_COMMUNICATION_PARAMETER_TAG_TABLE[parameter_index]) == 0) {
            break;
        }

    }

    if (parameter_index == NUMBER_OF_UDP_COMMUNICATION_PARAMETERS) {
        return;
    }

    if (string_array.size() < UDP_COMMUNICATION_PARAMETER_SIZE_TABLE[parameter_index] + 1) {
        return;
    }

    if (parameter_index == UDP_COMMUNICATION_CONFIGURATION_PARAMETER) {
        if ((ip_address.size() == 0) ||
            (port_number.size() == 0) ||
            (reception_ip_address_string.size() == 0) ||
            (reception_port_number_string.size() == 0)) {

            int temp_integer = 0;
            const int return_of_sscanf =
                sscanf(string_array.at(5).c_str(), "%d", &temp_integer);

            if ((return_of_sscanf != 0) &&
                (return_of_sscanf != EOF)) {
                *communication_timeout_usec = temp_integer;

                ip_address = string_array.at(1);
                port_number = string_array.at(2);

                reception_ip_address_string = string_array.at(3);
                reception_port_number_string = string_array.at(4);

                protocol_string = string_array.at(6);
            }

        }
    }

    return;
}

void decode_string_table_to_udp_communication_parameter(const std::vector< std::vector<std::string> > &string_table,
                                                        std::string &ip_address, std::string &port_number,
                                                        std::string &reception_ip_address_string,
                                                        std::string &reception_port_number_string,
                                                        int *communication_timeout_usec,
                                                        std::string &protocol_string)
{

    for (unsigned int i = 0; i < string_table.size(); ++i) {

        parse_string_to_udp_communication_parameter(string_table.at(i),
                                                    ip_address, port_number,
                                                    reception_ip_address_string,
                                                    reception_port_number_string,
                                                    communication_timeout_usec,
                                                    protocol_string);

    }

    return;
}
