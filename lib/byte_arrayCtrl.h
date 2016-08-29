#ifndef BYTE_ARRAY_CONTROL_H
#define BYTE_ARRAY_CONTROL_H
/*!
  \file
  \brief functions to handle byte array
  \author Kiyoshi MATSUO
  $Id
*/

// for NULL
#include <stddef.h>

// for std::vector
#include <vector>

// for ifstream
#include <fstream>

//! constants to handle byte array
enum CONSTANTS_FOR_BYTE_ARRAY_CONTROL {

    //! byte size
    BIT_SIZE_OF_BYTE = 8,

    //! invalid size of readed buffer
    INVALID_LENGTH_OF_READED_BYTE_FROM_CIRCULAR_BUFFER = -1,

    //! table size to convert byte array to value
    LENGTH_OF_HALF_MAXIMUM_VALUE_TABLE = 5,

    //! table size to convert value to byte array
    LENGTH_OF_MAXIMUM_VALUE_TABLE = 5,
};

//! half of maximum value of each byte
const unsigned int HALF_OF_EACH_BYTE_MAXIMUM_VALUE[LENGTH_OF_HALF_MAXIMUM_VALUE_TABLE] = {
    0, 0x80, 0x8000, 0x800000, 0x80000000
//128, 32768, 8388608, 2147483648
};

//! maximum value of each byte
const unsigned int EACH_BYTE_MAXIMUM_VALUE[LENGTH_OF_MAXIMUM_VALUE_TABLE] = {
    0, 0xFF, 0xFFFF, 0xFFFFFF, 0xFFFFFFFF
};

/*!
  \brief function to check whether is string bracketed
*/
extern bool is_bracketed_string(const char *byte_array, unsigned int array_length,
                                char bracket_code);

/*!
  \brief function to get bracket omitted string
*/
extern unsigned int get_bracket_omitted_string(const char *byte_array, unsigned int array_length,
                                               char bracket_code, std::string &omitted_string);

/*!
  \brief function to count specified code string in byte array
*/
extern unsigned int count_code_string_in_byte_array(const char *code, unsigned int code_length,
                                                    const char *byte_array, unsigned int byte_array_length);

/*!
  \brief function to check length of string
  \attention this function returns 0 if there are no any 0.
*/
extern unsigned int check_length_of_string(const char *string, unsigned int string_length);

/*!
  \brief function to evaluate which two strings are equal
*/
extern bool are_equal_strings(const char *string1, unsigned int string1_length,
                              const char *string2, unsigned int string2_length);

/*!
  \brief function to evaluate which the character is [0-9]
*/
extern bool is_decimal_character(const char *character);

/*!
  \brief function to evaluate which the character is [0-9]
*/
extern unsigned int count_length_of_continuous_decimal_characters(const char *character,
                                                                  unsigned int string_length);

/*!
  \brief function to convert decimal string to integer
  \attention this function uses sscanf
*/
extern bool convert_decimal_string_to_integer(const std::string &string, int *integer);

/*!
  \brief function to convert decimal character to unsigned integer
  \attention this function returns 0 if the character is not in [0-9]
*/
extern unsigned int convert_decimal_string_to_unsigned_integer(const char *string, unsigned int string_length,
                                                               bool big_endian);

/*!
  \brief function to convert decimal character to unsigned integer
  \attention this function returns 0 if the character is not in [0-9]
*/
extern unsigned int convert_continuous_decimal_string_to_unsigned_integer(const char *string, unsigned int string_length,
                                                                          bool big_endian);

/*!
  \brief function to convert unsigned integer to decimal string
*/
extern void convert_unsigned_integer_to_decimal_string(unsigned int value,
                                                       char *string_buffer, unsigned int buffer_length,
                                                       bool big_endian);

/*!
  \brief functiont to evaluate which the character is [0-9] or [A-F] or [a-f]
*/
extern bool is_hexadecimal_character(const char *character);

/*!
  \brief function to count number of hexadecimal characters
  \attention this function returns the smallest index of character which is not a hexadecimal character.
*/
extern unsigned int count_hexadecimal_characters(const char *string,
                                                 unsigned int string_length);

/*!
  \brief function to convert hexadecimal character to unsigned integer
  \attention this function returns 0 if character is not [0-9], [A-F], [a-f]
*/
extern unsigned int convert_hexadecimal_character_to_unsigned_integer(const char *character,
                                                                      unsigned int digit);

/*!
  \brief function to convert hexadecimal string to unsigned integer
  \attention this function returns 0 if the string is too long.
*/
extern unsigned int convert_hexadecimal_string_to_unsigned_integer(const char* string,
                                                                   int string_length,
                                                                   bool big_endian);

/*!
  \brief function to convert hexadecimal character to unsigned long
  \attention this function returns 0 if character is not [0-9], [A-F], [a-f]
*/
extern unsigned long convert_hexadecimal_character_to_unsigned_long(const char *character,
                                                                    unsigned int digit);

/*!
  \brief function to convert hexadecimal string to unsigned long
  \attention this function returns 0 if the string is too long.
*/
extern unsigned long convert_hexadecimal_string_to_unsigned_long(const char* string,
                                                                 int string_length,
                                                                 bool big_endian);

/*!
  \brief function to convert hexadecimal string to unsigned int array
  \return number of converted unsigned int
  \attention this function returns 0 if the string length is larger than sizeof(int)
  \attention this function uses OpenMP if possible
*/
extern int convert_hexadecimal_string_to_two_unsigned_integer_arrays(const char* string,
                                                                     unsigned int string_length,
                                                                     int one_string_length,
                                                                     bool big_endian,
                                                                     std::vector<unsigned int> &value_array1,
                                                                     std::vector<unsigned int> &value_array2);

/*!
  \brief function to evaluate which byte sequences are equals
*/
extern bool are_equivalent_byte_sequences(const char* sequence1, const char *sequence2,
                                          unsigned int evaluate_length_byte);

/*!
  \brief function to seek first code from byte array
  \attention this function return NULL if there are no codes in target message
*/
extern const char *seek_first_code_in_byte_array(const char *code, unsigned int code_length,
                                                 const char *message, unsigned int message_length);

/*!
  \brief function to seek first code from circular byte array
  \attention this function return NULL if there are no codes in target message
  \attention this function does work well if seek start or seek end is out from buffer
*/
extern const char* seek_first_code_from_circular_byte_array(const char *code, unsigned int code_length,
                                                            const char *buffer, unsigned int buffer_length,
                                                            const char *seek_start, const char *seek_end_out);

/*!
  \brief function to calculate circular length between pointers in circular buffer
  \attention this function does not work if pointer is not in circular buffer
*/
extern unsigned int calculate_circular_length_in_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                                 const char *start, const char *end_out);

/*!
  \brief function to calculate shifted pointer in circular buffer
  \attention this function does not work if pointer is not in circular buffer
*/
extern const char *calculate_shifted_constant_pointer_in_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                                         const char *pointer, unsigned int shift_amount);

/*!
  \brief function to calculate shifted pointer in circular buffer
  \attention this function does not work if pointer is not in circular buffer
*/
extern char *calculate_shifted_pointer_in_circular_buffer(char *buffer, unsigned int buffer_length,
                                                          char *pointer, unsigned int shift_amount);

/*!
  \brief function to calculate check pointer in the interval of circular buffer
  \attention this function does not work if some pointers are not in circular buffer
*/
extern bool is_pointer_in_interval_of_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                      const char *interval_start, const char *interval_end_out,
                                                      const char *pointer);

/*!
  \brief function to decode byte to unsigned value (endian selectable)
  \attention if length byte is larger than sizeof(unsigned int), this function returns decoded value of lower byte
  \attention this function does not check length of source
*/
extern unsigned int decode_unsigned_value(const char *buffer, unsigned int length_byte, bool big_endian);

/*!
  \brief function to decode unsigned value to byte array (endian selectable)
*/
extern void decode_unsigned_value_to_byte_array(unsigned int value,
                                                char *buffer, unsigned int length_byte,
                                                bool big_endian);

/*!
  \brief function to decode byte to signed value (endian selectable)
  \attention if length byte is larger than sizeof(int), this function does not work
  \attention this function does not check length of source
*/
extern int decode_signed_value(const char *buffer, unsigned int length_byte, bool big_endian);

/*!
  \brief function to decode unsigned value to byte array (endian selectable)
*/
extern void decode_signed_value_to_byte_array(int value,
                                              char *buffer, unsigned int length_byte,
                                              bool big_endian);

/*!
  \brief function to decode byte to unsigned value (little endian)
  \attention if length byte is larger than sizeof(unsigned int), this function returns decoded value of lower byte
  \attention this function does not check length of source
*/
extern unsigned int decode_unsigned_value(const char *buffer, unsigned int length_byte);

/*!
  \brief function to decode byte to signed value (little endian)
  \attention if length byte is larger than LENGTH_SIZE_OF_HALF_MAXIMUM_VALUE_TABLE, this function returns 0
  \attention this function does not check length of source
*/
extern int decode_signed_value(const char *buffer, unsigned int length_byte);

/*!
  \brief function to decode parts of circular byte array to unsigned value (little endian)
  \attention this function does not work if value start does not include the buffer
  \attention if length byte is larger than sizeof(unsigned int), this function returns decoded value of lower byte
*/
extern unsigned int decode_unsigned_value_from_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                               const char *value_start, unsigned int length_byte);

/*!
  \brief function to copy byte array from circular buffer
  \attention this function does not work if start is not in buffer
  \attention this function does not care length of destination buffer
*/
extern void copy_byte_from_circular_buffer(const char *buffer, unsigned int buffer_length,
                                           const char *start, unsigned int copy_length,
                                           char *destination);

/*!
  \brief function to copy byte array from circular buffer (inverse)
  \attention this function does not work if start is not in buffer
  \attention this function does not care length of destination buffer
*/
extern void copy_byte_from_circular_buffer_inversely(const char *buffer, unsigned int buffer_length,
                                                     const char *start, unsigned int copy_length,
                                                     char *destination);

/*!
  \brief function to decode unsigned value from circular buffer (big endian)
  \attention this function does not work if value start does not include the buffer
  \attention if length byte is larger than sizeof(unsigned int), this function returns decoded value of lower byte
*/
extern unsigned int convert_hexadecimal_string_from_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                                    const char *value_start, unsigned int length_byte, bool big_endian);

/*!
  \brief function to copy byte array to circular buffer
  \attention this function does not work if start is not in buffer
  \attention this function does not check overwrite in circular buffer
*/
extern void copy_byte_to_circular_buffer(const char *source, unsigned int source_length,
                                         char *buffer, unsigned int buffer_length,
                                         char *destination_start);


/*!
  \brief function to read file to circular buffer
  \attention this function does not work if either start or end_out is not in buffer
  \attention this function returns -1 if read error occurred
  \attention this function uses dynamic memory allocated memory as temporal buffer
*/
extern int read_stream_to_circular_buffer(std::ifstream &stream,
                                          char *buffer, unsigned int buffer_length,
                                          char *start, char *end_out);

/*!
  \brief function to seek codes on circular buffer
*/
extern unsigned int seek_codes_from_circular_byte_array(const char *code, unsigned int code_length,
                                                        const char *buffer, unsigned int buffer_length,
                                                        const char **seek_start, const char *seek_end_out,
                                                        const char **code_array, unsigned int code_array_length);

/*!
  \brief function to copy parsed byte array to byte table
  \attention this function does not work if the table size is less than number of code
*/
extern unsigned int copy_tail_code_parsed_byte_array(const char *buffer, unsigned int buffer_length,
                                                     const char **past_copied_end_out,
                                                     const char **code_array, unsigned int code_length,
                                                     unsigned int number_of_code, unsigned int code_array_length,
                                                     char *byte_table, unsigned int size_of_one_line_in_table,
                                                     unsigned int *received_length_array);

/*!
  \brief function to copy byte array to string
*/
extern void copy_byte_to_string(const char *buffer, unsigned int buffer_length,
                                std::string &destination);

/*!
  \brief function to parse separated byte array to string array
*/
extern unsigned int parse_separated_byte_array(const char *buffer, unsigned int buffer_length,
                                               char separator,
                                               std::vector<std::string> &parsed_string_array);

//! structure for circular buffer
struct circular_buffer_t {

    //! buffer size [byte]
    unsigned int length_byte;

    //! remaining empty buffer length
    unsigned int empty_buffer_length_byte;

    //! buffer pointer
    char *data_start_point;

    //! copy destination
    char *destination_point;

    //! data used end out pointer
    char *used_data_end_out_point;

};

/*!
  \brief function to initialize circular buffer
  \attention this function set data_start_point, destination_point, and used_data_point are NULL.
*/
extern void initialize_circular_buffer(circular_buffer_t *buffer);

/*!
  \brief function to clear circular buffer
*/
extern void clear_memory_of_circular_buffer(circular_buffer_t *buffer);

/*!
  \brief function to allocate memory for circular buffer
  \attention this function does not work if the buffer is not initialized
  \attention this function returns false if the buffer_length_byte is zero
*/
extern bool allocate_memory_for_circular_buffer(circular_buffer_t *buffer, unsigned int buffer_length_byte);

/*!
  \brief function to release memory of circular buffer
  \attention this function does not work if the buffer size is zero
  \attention this functioin returns false if the buffer size is zero or pointer is null
*/
extern bool release_memory_of_circular_buffer(circular_buffer_t *buffer);

/*!
  \brief function to get length of empty data buffer
*/
extern unsigned int get_length_of_empty_data_buffer(const circular_buffer_t *buffer);

/*!
  \brief function to copy data to circular buffer
  \attention this function copy data which is equals empty buffer length when the empty size is not enough
*/
extern unsigned int copy_byte_to_circular_buffer(const char *source, unsigned int source_length,
                                                 circular_buffer_t *buffer);

//! enumeration for copy mode
enum ENUMERATION_FOR_BYTE_COPY_MODE {

    //! invalid byte copy mode
    INVALID_BYTE_COPY_MODE = -1,

    //! write all data (overwrite)
    BYTE_COPY_MODE_OVERWRITE = 0,

    //! write all data (overwite but prohibit partial write, if the buffer size is not enough, function does not write any data)
    BYTE_COPY_MODE_OVERWRITE_PROHIBIT_PARTIAL_WRITE,

    //! write possible data (non overwite)
    BYTE_COPY_MODE_POSSIBLE_ALL_WRITE,

    //! write only set data (if the empty buffer is not enough, function does not write any data)
    BYTE_COPY_MODE_PROHIBIT_PARTIAL_WRITE,

    //! read only set data (if the remaining data in circular buffer is not enough, function does not read any data)
    BYTE_COPY_MODE_PROHIBIT_PARTIAL_READ,

    //! read possible all data (if the remaining data size is smaller than reading size, function does read all the remaining data. the remaining data size is larger than reading size, function does read only reading size)
    BYTE_COPY_MODE_POSSIBLE_ALL_READ,

    //! number of copy mode
    NUMBER_OF_MODES_FOR_BYTE_COPY,
};

/*!
  \brief function to get data byte length to copy to circular buffer
*/
extern unsigned int get_data_byte_length_to_copy_to_circular_buffer(enum ENUMERATION_FOR_BYTE_COPY_MODE copy_mode,
                                                                    const circular_buffer_t *buffer,
                                                                    unsigned int source_length);

/*!
  \brief function to copy data to circular buffer
*/
extern unsigned int copy_byte_to_circular_buffer(const char *source, unsigned int source_length,
                                                 circular_buffer_t *buffer, enum ENUMERATION_FOR_BYTE_COPY_MODE copy_mode);

/*!
  \brief function to calculate length of remaining data
*/
extern unsigned int calculate_remaining_data_length(const circular_buffer_t *buffer, const char *start);

/*!
  \brief function to copy data from circular buffer
  \attention this function does not care the destination buffer size
*/
extern unsigned int copy_byte_from_circular_buffer(const circular_buffer_t *buffer,
                                                   const char *source, unsigned int source_length,
                                                   char *destination, enum ENUMERATION_FOR_BYTE_COPY_MODE copy_mode);
/*!
  \brief function to calculate length of remaining data buffer
*/
extern unsigned int calculate_remaining_data_length(const circular_buffer_t *buffer);

/*!
  \brief function to move used data end out pointer
  \attention this function does not check renewed_used_data_end_out is in the buffer
*/
extern void move_used_data_end_out_point(circular_buffer_t *buffer,
                                         unsigned int move_amount);

/*!
  \brief function to move used data end out pointer
  \return moved size
  \attention this function does not move the pointer if no data exists
  \attention this function does not move the pointer if the used_data_end_out is not the non used data pointer
*/
extern unsigned int move_used_data_end_out_point(circular_buffer_t *buffer,
                                                 const char *used_data_end_out);

/*!
  \brief function to copy data from circular buffer
*/
extern unsigned int copy_non_used_data_byte_from_circular_buffer(circular_buffer_t *source, unsigned int source_length,
                                                                 char *destination, unsigned int destination_length);

/*!
  \brief function to copy first tailed byte array from circular buffer
*/
extern unsigned int copy_first_tailed_byte_array_from_circular_buffer(circular_buffer_t *buffer,
                                                                      const char *code, unsigned int code_length,
                                                                      char *destination_buffer, unsigned int destination_buffer_length,
                                                                      unsigned int *tailed_byte_array_length);

/*!
  \brief function to shift byte data (remove head data)
*/
extern void shift_byte_data(char *buffer, unsigned int buffer_length,
                            int shift_amount, char fill_data);

/*!
  \brief function to seek template partial match
*/
extern unsigned int seek_template_forward_match(const char *template_array, unsigned int template_length,
                                                const char *data_array, unsigned int data_length);

/*!
  \brief function to shift forward matching byte data
*/
extern unsigned int shift_forward_matching_byte_data(char *data, unsigned int data_length, char fill_data,
                                                     const char *template_array, unsigned int template_length);

#endif // BYTE_ARRAY_CONTROL_H
