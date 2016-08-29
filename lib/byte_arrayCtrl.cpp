
#include "byte_arrayCtrl.h"

// for malloc free
#include <stdlib.h>

// for memcpy
#include <string.h>

// for debug
#include <iostream>

bool is_bracketed_string(const char *byte_array, unsigned int array_length,
                         char bracket_code)
{
    if (array_length <= 1) {
        return false;
    }

    if ((byte_array[0] == bracket_code) &&
        (byte_array[array_length - 1] == bracket_code)) {

        return true;

    }

    return false;
}

unsigned int get_bracket_omitted_string(const char *byte_array, unsigned int array_length,
                                        char bracket_code, std::string &omitted_string)
{
    if (omitted_string.size() > 0) {
        omitted_string.clear();
    }

    unsigned int copy_length = array_length;
    const char *source_start_point = byte_array;

    if (is_bracketed_string(byte_array, array_length, bracket_code) == true) {

        copy_length = array_length - 2;
        source_start_point = byte_array + 1;

    }

    omitted_string.resize(copy_length, 0);
    for (unsigned int i = 0; i < copy_length; ++i) {
        omitted_string.at(i) = *(source_start_point + i);
    }

    return omitted_string.size();
}

unsigned int count_code_string_in_byte_array(const char *code, unsigned int code_length,
                                             const char *byte_array, unsigned int byte_array_length)
{
    unsigned int number_of_codes = 0;

    unsigned int discovered_character_count = 0;
    const char *next_seek_character = code + discovered_character_count;

    unsigned int i = 0;

    for (i = 0; i < byte_array_length; ++i) {

        if (byte_array[i] == (*next_seek_character)) {
            ++discovered_character_count;
        } else {
            discovered_character_count = 0;
        }

        if (discovered_character_count == code_length) {
            ++number_of_codes;
            discovered_character_count = 0;
        }

        next_seek_character = code + discovered_character_count;

    }

    return number_of_codes;
}

unsigned int check_length_of_string(const char *string, unsigned int string_length)
{

    for (unsigned int index = 0; index < string_length; ++index) {

        if (string[index] == 0) {
            return index;
        }
    }

    return 0;
}

bool are_equal_strings(const char *string1, unsigned int string1_length,
                       const char *string2, unsigned int string2_length)
{

    const unsigned int length1 = check_length_of_string(string1, string1_length);
    const unsigned int length2 = check_length_of_string(string2, string2_length);

    if ((length1 == 0) ||
        (length2 == 0)) {
        return false;
    }

    if (length1 != length2) {
        return false;
    }

    for (unsigned int i = 0; i < length1; ++i) {

        if (string1[i] != string2[i]) {
            return false;
        }

    }

    return true;
}

bool is_decimal_character(const char *character)
{

   if ((*character >= '0') &&
       (*character <= '9')) {

        return true;
   }

   return false;
}


unsigned int count_length_of_continuous_decimal_characters(const char *character, unsigned int string_length)
{
    unsigned int length = 0;

    for (unsigned int i = 0; i < string_length; ++i) {

        if (is_decimal_character(character + i) == false) {
            break;
        }
        ++length;

    }

    return length;
}

bool convert_decimal_string_to_integer(const std::string &string, int *integer)
{

    int temp_integer = 0;

    const int return_of_sscanf =
        sscanf(string.c_str(), "%d", &temp_integer);

    if ((return_of_sscanf != 0) &&
        (return_of_sscanf != EOF)) {
        *integer = temp_integer;

        return true;
    }

    return false;
}

unsigned int convert_decimal_string_to_unsigned_integer(const char *string, unsigned int string_length,
                                                        bool big_endian)
{
    unsigned int value = 0;
    bool all_decimal_characters = true;

    unsigned int digit_order = 1;

    if (big_endian == true) {
        for (unsigned int i = 0; i < string_length; ++i) {
            if (is_decimal_character(string + i) == false) {
                all_decimal_characters = false;
                break;
            }
            value += digit_order * (unsigned int)(string[i] - '0');
            digit_order = 10 * digit_order;
        }
    } else {
        for (unsigned int i = 0; i < string_length; ++i) {
            if (is_decimal_character(string + string_length - i - 1) == false) {
                all_decimal_characters = false;
                break;
            }
            value += digit_order * (unsigned int)(string[string_length - i - 1] - '0');
            digit_order = 10 * digit_order;
        }
    }

    if (all_decimal_characters == false) {
        value = 0;
    }

    return value;
}

unsigned int convert_continuous_decimal_string_to_unsigned_integer(const char *string, unsigned int string_length,
                                                                   bool big_endian)
{
    unsigned int value = 0;

    unsigned int digit_order = 1;

    if (big_endian == true) {
        for (unsigned int i = 0; i < string_length; ++i) {
            if (is_decimal_character(string + i) == false) {
                break;
            }
            value += digit_order * (unsigned int)(string[i] - '0');
            digit_order = 10 * digit_order;
        }
    } else {

        const unsigned int length =
            count_length_of_continuous_decimal_characters(string, string_length);

        for (unsigned int i = 0; i < length; ++i) {
            if (is_decimal_character(string + length - i - 1) == false) {
                break;
            }
            value += digit_order * (unsigned int)(string[length - i - 1] - '0');
            digit_order = 10 * digit_order;
        }
    }

    return value;
}

void convert_unsigned_integer_to_decimal_string(unsigned int value,
                                                char *string_buffer, unsigned int buffer_length,
                                                bool big_endian)
{
    unsigned int temporal_value = value;
    char one_digit_value = 0;
    if (big_endian == true) {
        for (unsigned int i = 0; i < buffer_length; ++i) {

            one_digit_value = (char)(temporal_value % 10);
            temporal_value = temporal_value / 10;

            string_buffer[i] = one_digit_value + '0';

        }

    } else {

        for (unsigned int i = 0; i < buffer_length; ++i) {

            one_digit_value = (char)(temporal_value % 10);
            temporal_value = temporal_value / 10;

            string_buffer[buffer_length - 1 - i] = one_digit_value + '0';

        }

    }

    return;
}

bool is_hexadecimal_character(const char *character)
{

    if ((*character >= '0') &&
        (*character <= '9')) {

        return true;

    } else if ((*character >= 'a') &&
               (*character <= 'f')) {

        return true;

    } else if ((*character >= 'A') &&
               (*character <= 'F')) {

        return true;

    }

    return false;
}

unsigned int count_hexadecimal_characters(const char *string,
                                          unsigned int string_length)
{
    unsigned int counter = 0;

    for (counter = 0; counter < string_length; ++counter) {

        if (is_hexadecimal_character(&string[counter]) == false) {
            break;
        }
    }

    return counter;
}

unsigned int convert_hexadecimal_character_to_unsigned_integer(const char *character,
                                                               unsigned int digit)
{
    unsigned int value = 0;

    if ((*character >= '0') &&
        (*character <= '9')) {

        value = (unsigned int)(*character - '0');

    } else if ((*character >= 'a') &&
               (*character <= 'f')) {

        value = (unsigned int)(*character - 'a') + 10;

    } else if ((*character >= 'A') &&
               (*character <= 'F')) {

        value = (unsigned int)(*character - 'A') + 10;

    }

    const unsigned int bit_shift_amount = (digit << 2);

    return (value << bit_shift_amount);

}

unsigned int convert_hexadecimal_string_to_unsigned_integer(const char *string,
                                                            int string_length,
                                                            bool big_endian)
{
    unsigned int value = 0;

    if ((string_length <= 0 ) ||
        (string_length > (int)sizeof(unsigned int) * 2)) {

        return value;
    }

    const char *reading_pointer = string;

    if (big_endian == true) {

        for (int i = 0; i < string_length; ++i) {
            value += convert_hexadecimal_character_to_unsigned_integer(reading_pointer,
                                                                       string_length - i - 1);

            ++reading_pointer;
        }

    } else {

        for (int i = 0; i < string_length; ++i) {
            value += convert_hexadecimal_character_to_unsigned_integer(reading_pointer,
                                                                       i);
            ++reading_pointer;
        }

    }

    return value;
}

unsigned long convert_hexadecimal_character_to_unsigned_long(const char *character,
                                                             unsigned int digit)
{
    unsigned long value = 0;

    if ((*character >= '0') &&
        (*character <= '9')) {

        value = (unsigned long)(*character - '0');

    } else if ((*character >= 'a') &&
               (*character <= 'f')) {

        value = (unsigned long)(*character - 'a') + 10;

    } else if ((*character >= 'A') &&
               (*character <= 'F')) {

        value = (unsigned long)(*character - 'A') + 10;

    }

    const unsigned int bit_shift_amount = (digit << 2);

    return (value << bit_shift_amount);

}

unsigned long convert_hexadecimal_string_to_unsigned_long(const char *string,
                                                          int string_length,
                                                          bool big_endian)
{
    unsigned long value = 0;

    if ((string_length <= 0 ) ||
        (string_length > (int)sizeof(unsigned long))) {

        return value;
    }

    const char *reading_pointer = string;

    if (big_endian == true) {

        for (int i = 0; i < string_length; ++i) {
            value += convert_hexadecimal_character_to_unsigned_long(reading_pointer,
                                                                    string_length - i - 1);

            ++reading_pointer;
        }

    } else {

        for (int i = 0; i < string_length; ++i) {
            value += convert_hexadecimal_character_to_unsigned_long(reading_pointer,
                                                                    i);
            ++reading_pointer;
        }

    }

    return value;
}

int convert_hexadecimal_string_to_two_unsigned_integer_arrays(const char* string,
                                                              unsigned int string_length,
                                                              int one_string_length,
                                                              bool big_endian,
                                                              std::vector<unsigned int> &value_array1,
                                                              std::vector<unsigned int> &value_array2)
{

    const int conversion_buffer_length = count_hexadecimal_characters(string,
                                                                      string_length);

    const int number_of_data = conversion_buffer_length / (2 * one_string_length);

    if ((int)value_array1.size() != number_of_data) {
        value_array1.clear();
        value_array1.resize(number_of_data, 0);
    }

    if ((int)value_array2.size() != number_of_data) {
        value_array2.clear();
        value_array2.resize(number_of_data, 0);
    }

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < number_of_data; ++i) {

        const char *reading_buffer = string + 2 * one_string_length * i;

        value_array1.at(i) =
            convert_hexadecimal_string_to_unsigned_integer(reading_buffer, one_string_length,
                                                           big_endian);


        reading_buffer += one_string_length;

        value_array2.at(i) =
            convert_hexadecimal_string_to_unsigned_integer(reading_buffer, one_string_length,
                                                           big_endian);

    }


    return number_of_data;
}

bool are_equivalent_byte_sequences(const char* sequence1, const char *sequence2,
                                   unsigned int evaluate_length_byte)
{

    for (unsigned int i = 0; i < evaluate_length_byte; ++i) {

        if (sequence1[i] != sequence2[i]) {
            return false;
        }

    }

    return true;
}

const char *seek_first_code_in_byte_array(const char *code, unsigned int code_length,
                                          const char *message, unsigned int message_length)
{

    const char *packet_start_point = NULL;

    unsigned int discovered_character_count = 0;
    const char *next_seek_character = code + discovered_character_count;

    unsigned int i = 0;
    for (i = 0; i < message_length; ++i) {

        if (message[i] == (*next_seek_character)) {
            ++discovered_character_count;
            next_seek_character = code + discovered_character_count;
        } else {
            discovered_character_count = 0;
            next_seek_character = code + discovered_character_count;
        }

        if (discovered_character_count == code_length) {
            break;
        }

    }

    if (discovered_character_count == code_length) {
        packet_start_point = &message[i + 1 - code_length];
    }

    return packet_start_point;

}

const char* seek_first_code_from_circular_byte_array(const char *code, unsigned int code_length,
                                                     const char *buffer, unsigned int buffer_length,
                                                     const char *seek_start, const char *seek_end_out)
{
    const char *packet_start_point = NULL;
    const char *buffer_end_out = buffer + buffer_length;

    if (seek_start < seek_end_out) {

        const unsigned int seek_width = seek_end_out - seek_start;
        packet_start_point = seek_first_code_in_byte_array(code, code_length,
                                                           seek_start, seek_width);

    } else {
        // cyclic seek
        const unsigned int seek_width_to_buffer_end = buffer_end_out - seek_start;

        unsigned int discovered_character_count = 0;
        const char *next_seek_character = code + discovered_character_count;

        unsigned int i = 0;
        for (i = 0; i < seek_width_to_buffer_end; ++i) {

            if (seek_start[i] == (*next_seek_character)) {
                ++discovered_character_count;
                next_seek_character = code + discovered_character_count;
            } else {
                discovered_character_count = 0;
                next_seek_character = code + discovered_character_count;
            }

            if (discovered_character_count == code_length) {
                break;
            }

        }

        if (discovered_character_count == code_length) {

            packet_start_point = &seek_start[i + 1 - code_length];

        } else {

            const char *second_seek_start = buffer;
            unsigned int second_seek_width = 0;

            if (discovered_character_count > 0) {
                const unsigned int remaining_count = code_length - discovered_character_count;

                second_seek_width = seek_end_out - buffer;

                if (seek_first_code_in_byte_array(&code[discovered_character_count], remaining_count,
                                                  second_seek_start, second_seek_width) != NULL) {

                    packet_start_point = buffer_end_out - discovered_character_count;

                }

            }

            if (packet_start_point == NULL) {

                second_seek_start = buffer;
                second_seek_width = seek_end_out - buffer;

                packet_start_point = seek_first_code_in_byte_array(code, code_length,
                                                                   second_seek_start, second_seek_width);
            }

        }


    }

    return packet_start_point;
}

unsigned int calculate_circular_length_in_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                          const char *start, const char *end_out)
{

    if (start < end_out) {
        return end_out - start;
    }

    return (buffer + buffer_length - start) + (end_out - buffer);
}

const char *calculate_shifted_constant_pointer_in_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                                  const char *pointer, unsigned int shift_amount)
{
    const char *shifted_pointer = pointer + shift_amount;

    if (shifted_pointer >= buffer + buffer_length) {
        shifted_pointer = shifted_pointer - buffer_length;
    }

    return shifted_pointer;
}

char *calculate_shifted_pointer_in_circular_buffer(char *buffer, unsigned int buffer_length,
                                                   char *pointer, unsigned int shift_amount)
{
    char *shifted_pointer = pointer + shift_amount;

    if (shifted_pointer >= buffer + buffer_length) {
        shifted_pointer = shifted_pointer - buffer_length;
    }

    return shifted_pointer;
}

bool is_pointer_in_interval_of_circular_buffer(const char *buffer, unsigned int buffer_length,
                                               const char *interval_start, const char *interval_end_out,
                                               const char *pointer)
{
    if (interval_start < interval_end_out) {

        return ((pointer >= interval_start) &&
                (pointer < interval_end_out));

    }

    return (((pointer >= interval_start) &&
             (pointer < buffer + buffer_length)) ||
            ((pointer >= buffer) &&
             (pointer < interval_end_out)));
}

unsigned int decode_unsigned_value(const char *buffer, unsigned int length_byte, bool big_endian)
{

    unsigned int value = 0;

    unsigned int byte_count = 0;
    unsigned int bit_shift_amount = 0;

    unsigned int scaled_length_byte = length_byte;
    if (length_byte > sizeof(unsigned int)) {
        scaled_length_byte = sizeof(unsigned int);
    }

    if (big_endian == false) {

        for (unsigned int i = 0; i < length_byte; ++i) {

            value += (((unsigned int)((unsigned char)*(buffer + i))) << bit_shift_amount);

            ++byte_count;
            if (byte_count == scaled_length_byte) {
                break;
            }
            bit_shift_amount += BIT_SIZE_OF_BYTE;

        }

    } else {

        for (unsigned int i = 0; i < length_byte; ++i) {

            value += (((unsigned int)((unsigned char)*(buffer + length_byte - 1 - i))) << bit_shift_amount);

            ++byte_count;
            if (byte_count == scaled_length_byte) {
                break;
            }
            bit_shift_amount += BIT_SIZE_OF_BYTE;

        }

    }

    return value;

}

void decode_unsigned_value_to_byte_array(unsigned int value,
                                         char *buffer, unsigned int length_byte, bool big_endian)
{
    memset((void *)buffer, 0, length_byte);

    if (length_byte >= LENGTH_OF_MAXIMUM_VALUE_TABLE) {
        return;
    }

    const unsigned int scaled_value =
        value & EACH_BYTE_MAXIMUM_VALUE[length_byte];

    unsigned int temp_value = scaled_value;
    char byte = 0;

    if (big_endian == false) {

        for (unsigned int i = 0; i < length_byte; ++i) {

            byte = (char)(unsigned char)(temp_value & 0xFF);
            temp_value = temp_value >> BIT_SIZE_OF_BYTE;

            buffer[i] = byte;

        }

    } else {

        for (unsigned int i = 0; i < length_byte; ++i) {

            byte = (char)(unsigned char)(temp_value & 0xFF);
            temp_value = temp_value >> BIT_SIZE_OF_BYTE;

            buffer[length_byte - 1 - i] = byte;

        }

    }

    return;
}

int decode_signed_value(const char *buffer, unsigned int length_byte, bool big_endian)
{
    if (length_byte >= LENGTH_OF_HALF_MAXIMUM_VALUE_TABLE) {
        return 0;
    }

    int signed_value = 0;
    unsigned int unsigned_value =
        decode_unsigned_value(buffer, length_byte, big_endian);

    if (unsigned_value >= HALF_OF_EACH_BYTE_MAXIMUM_VALUE[length_byte]) {
        unsigned_value -= HALF_OF_EACH_BYTE_MAXIMUM_VALUE[length_byte];
        signed_value = ((int)unsigned_value) - HALF_OF_EACH_BYTE_MAXIMUM_VALUE[length_byte];

    } else {

        signed_value = (int)unsigned_value;

    }

    return signed_value;
}

void decode_signed_value_to_byte_array(int value,
                                       char *buffer, unsigned int length_byte,
                                       bool big_endian)
{
    memset((void *)buffer, 0, length_byte);

    if (length_byte >= LENGTH_OF_MAXIMUM_VALUE_TABLE) {
        return;
    }

    if ((value > (int)EACH_BYTE_MAXIMUM_VALUE[length_byte]) ||
        (value < -(int)EACH_BYTE_MAXIMUM_VALUE[length_byte])) {
        return;
    }

    unsigned int converted_value = 0;

    if (value >= 0) {
        converted_value = (unsigned int)value;
    } else {
        converted_value = (unsigned int)(value + EACH_BYTE_MAXIMUM_VALUE[length_byte]);
    }

    decode_unsigned_value_to_byte_array(converted_value, buffer, length_byte,
                                        big_endian);
    return;
}

unsigned int decode_unsigned_value(const char *buffer, unsigned int length_byte)
{
    unsigned int value = 0;

    unsigned int byte_count = 0;
    unsigned int bit_shift_amount = 0;

    for (unsigned int i = 0; i < length_byte; ++i) {

        value += (((unsigned int)((unsigned char)*(buffer + i))) << bit_shift_amount);

        ++byte_count;
        if (byte_count == length_byte) {
            break;
        }
        bit_shift_amount += BIT_SIZE_OF_BYTE;

    }

    return value;
}

int decode_signed_value(const char *buffer, unsigned int length_byte)
{
    if (length_byte >= LENGTH_OF_HALF_MAXIMUM_VALUE_TABLE) {
        return 0;
    }

    int signed_value = 0;
    unsigned int unsigned_value =
        decode_unsigned_value(buffer, length_byte);

    if (unsigned_value >= HALF_OF_EACH_BYTE_MAXIMUM_VALUE[length_byte]) {
        unsigned_value -= HALF_OF_EACH_BYTE_MAXIMUM_VALUE[length_byte];
        signed_value = ((int)unsigned_value) - HALF_OF_EACH_BYTE_MAXIMUM_VALUE[length_byte];

    } else {

        signed_value = (int)unsigned_value;

    }

    return signed_value;
}

unsigned int decode_unsigned_value_from_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                        const char *value_start, unsigned int length_byte)
{
    unsigned int value = 0;

    const char *array_end_out =
        calculate_shifted_constant_pointer_in_circular_buffer(buffer, buffer_length,
                                                              value_start, length_byte);

    if (value_start < array_end_out) {

        unsigned int byte_count = 0;
        unsigned int bit_shift_amount = 0;

        for (const char *seek_point = value_start; seek_point < array_end_out; ++seek_point) {

            value += (((unsigned int)((unsigned char)*seek_point)) << bit_shift_amount);

            ++byte_count;
            if (byte_count == length_byte) {
                break;
            }
            bit_shift_amount += BIT_SIZE_OF_BYTE;

        }

    } else {

        const char *buffer_end_out = buffer + buffer_length;
        unsigned int byte_count = 0;
        unsigned int bit_shift_amount = 0;
        for (const char *seek_point = value_start; seek_point < buffer_end_out; ++seek_point) {

            value += (((unsigned int)((unsigned char)*seek_point)) << bit_shift_amount);

            ++byte_count;
            if (byte_count == length_byte) {
                break;
            }
            bit_shift_amount += BIT_SIZE_OF_BYTE;

        }

        if (byte_count < length_byte) {
            for (const char *seek_point = buffer; seek_point < array_end_out; ++seek_point) {

                value += (((unsigned int)((unsigned char)*seek_point)) << bit_shift_amount);

                ++byte_count;
                if (byte_count == length_byte) {
                    break;
                }
                bit_shift_amount += BIT_SIZE_OF_BYTE;

            }
        }

    }

    return value;
}

void copy_byte_from_circular_buffer(const char *buffer, unsigned int buffer_length,
                                    const char *start, unsigned int copy_length,
                                    char *destination)
{

    if (start + copy_length <= buffer + buffer_length) {

        memcpy((void *)destination, (void *)start,
               copy_length);

    } else {

        const unsigned int length_to_end_out_of_buffer =
            buffer + buffer_length - start;

        const unsigned int remaining_length =
            copy_length - length_to_end_out_of_buffer;

        memcpy((void *)destination, (void *)start,
               length_to_end_out_of_buffer);

        memcpy((void*)(destination + length_to_end_out_of_buffer), buffer,
               remaining_length);

    }

    return;
}

void copy_byte_from_circular_buffer_inversely(const char *buffer, unsigned int buffer_length,
                                              const char *start, unsigned int copy_length,
                                              char *destination)
{
    copy_byte_from_circular_buffer(buffer, buffer_length, start, copy_length, destination);

    const unsigned int copy_length_half = copy_length / 2;

    for (unsigned int i = 0; i < copy_length_half; ++i) {

        char temp_char = destination[i];

        destination[i] = destination[copy_length - 1 - i];
        destination[copy_length - 1 - i] = temp_char;

    }

    return;
}

unsigned int convert_hexadecimal_string_from_circular_buffer(const char *buffer, unsigned int buffer_length,
                                                             const char *value_start, unsigned int length_byte, bool big_endian)
{
    unsigned int unsigned_value = 0;
    char *temporal_buffer = (char *)malloc(length_byte * sizeof(char));
    if (temporal_buffer == NULL) {
        return unsigned_value;
    }

    copy_byte_from_circular_buffer(buffer, buffer_length,
                                   value_start, length_byte,
                                   temporal_buffer);

    memset((void *)&unsigned_value, 0, sizeof(unsigned_value));

    unsigned_value = convert_hexadecimal_string_to_unsigned_integer(temporal_buffer, length_byte, big_endian);

    free(temporal_buffer);

    return unsigned_value;
}

void copy_byte_to_circular_buffer(const char *source, unsigned int source_length,
                                  char *buffer, unsigned int buffer_length,
                                  char *destination_start)
{
    const char *temporal_source = source;
    unsigned int remaining_length = source_length;

    char *destination = destination_start;
    char *buffer_end_out = buffer + buffer_length;

    while (1) {

        if (destination + remaining_length > buffer_end_out) {

            const unsigned int copy_size = buffer_end_out - destination;
            memcpy((void *)destination,
                   (void *)temporal_source, copy_size);

            remaining_length -= copy_size;

            temporal_source = temporal_source + copy_size;
            destination = calculate_shifted_pointer_in_circular_buffer(buffer, buffer_length,
                                                                       destination, copy_size);

        } else {

            memcpy((void *)destination, (void *)temporal_source,
                   remaining_length);

            remaining_length = 0;

            break;

        }

    }

    return;
}

int read_stream_to_circular_buffer(std::ifstream &stream,
                                   char *buffer, unsigned int buffer_length,
                                   char *start, char *end_out)
{
    int read_size = 0;

    const unsigned int current_buffer_size =
        calculate_circular_length_in_circular_buffer(buffer, buffer_length,
                                                     start, end_out);

    if (current_buffer_size == 0) {
        return read_size;
    }

    if (stream.eof() == true) {
        return read_size;
    }

    char *temporal_buffer = (char *)malloc(sizeof(char) * current_buffer_size);

    stream.read(temporal_buffer, current_buffer_size);

    if (stream.bad() == true) {
        free(temporal_buffer);
        return INVALID_LENGTH_OF_READED_BYTE_FROM_CIRCULAR_BUFFER;
    }

    read_size = stream.gcount();

    if (start < end_out) {

        memcpy((void *)start, (void *)temporal_buffer,
               current_buffer_size);

    } else {

        const unsigned int length_of_first_part = buffer + buffer_length - start;
        const unsigned int length_of_second_part = end_out - buffer;

        memcpy((void *)start, (void *)temporal_buffer,
               length_of_first_part);

        if (length_of_second_part > 0) {
            memcpy((void *)buffer, (void *)(&temporal_buffer[length_of_first_part]),
                   length_of_second_part);
        }

    }

    free(temporal_buffer);

    return read_size;
}

unsigned int seek_codes_from_circular_byte_array(const char *code, unsigned int code_length,
                                                 const char *buffer, unsigned int buffer_length,
                                                 const char **seek_start, const char *seek_end_out,
                                                 const char **code_array, unsigned int code_array_length)
{
    unsigned int number_of_code = 0;

    const char **temp_code_array = (const char **)malloc(sizeof(const char *) * code_array_length);
    if (temp_code_array == NULL) {
        return number_of_code;
    }
    unsigned int code_set_index = 0;

    while (1) {
        const char *code_point = seek_first_code_from_circular_byte_array(code, code_length,
                                                                          buffer, buffer_length,
                                                                          *seek_start,
                                                                          seek_end_out);

        if (code_point == NULL) {
            break;
        } else {

            temp_code_array[code_set_index] = code_point;
            ++number_of_code;
            ++code_set_index;

            if (code_set_index >= code_array_length) {
                code_set_index = 0;
            }

            const int remaining_length =
                ((int)calculate_circular_length_in_circular_buffer(buffer, buffer_length,
                                                                   code_point, seek_end_out) - (int)code_length);

            *seek_start =
                calculate_shifted_constant_pointer_in_circular_buffer(buffer, buffer_length,
                                                                      code_point, code_length);


            if (remaining_length < (int)code_length) {
                break;
            }

        }

    }

    if (number_of_code > code_array_length) {

        for (unsigned int i = 0; i < code_set_index; ++i) {
            code_array[code_array_length - 1 - i] = temp_code_array[code_set_index - 1 - i];
        }

        for (unsigned int i = code_set_index; i < code_array_length; ++i) {
            code_array[code_array_length - 1 - i] = temp_code_array[code_set_index + code_array_length - 1 - i];
        }

    } else {

        for (unsigned int i = 0; i < number_of_code; ++i) {
            code_array[i] = temp_code_array[i];
        }

        for (unsigned int i = number_of_code; i < code_array_length; ++i) {
            code_array[i] = NULL;
        }
    }

    free(temp_code_array);

    return number_of_code;
}

unsigned int copy_tail_code_parsed_byte_array(const char *buffer, unsigned int buffer_length,
                                              const char **past_copied_end_out,
                                              const char **code_array, unsigned int code_length,
                                              unsigned int number_of_code, unsigned int code_array_length,
                                              char *byte_table, unsigned int size_of_one_line_in_table,
                                              unsigned int *received_length_array)
{
    unsigned int total_copied_size = 0;
    unsigned int one_string_length = 0;

    const char *copy_start = NULL;

    if (number_of_code == 1) {

        copy_start = *past_copied_end_out;

        one_string_length =
            calculate_circular_length_in_circular_buffer(buffer, buffer_length,
                                                         *past_copied_end_out, code_array[0]) + code_length;

        copy_byte_from_circular_buffer(buffer, buffer_length,
                                       copy_start, one_string_length,
                                       byte_table);

        received_length_array[0] = one_string_length;

        total_copied_size = one_string_length;

        *past_copied_end_out =
            calculate_shifted_constant_pointer_in_circular_buffer(buffer, buffer_length,
                                                                  copy_start, one_string_length);

    } else if (number_of_code <= code_array_length) {

        for (unsigned i = 0; i < number_of_code - 1; ++i) {

            one_string_length =
                calculate_circular_length_in_circular_buffer(buffer, buffer_length,
                                                             code_array[number_of_code - 1 - i - 1],
                                                             code_array[number_of_code - 1 - i]);

            copy_start =
                calculate_shifted_constant_pointer_in_circular_buffer(buffer, buffer_length,
                                                                      code_array[number_of_code - 1 - i - 1],
                                                                      code_length);

            copy_byte_from_circular_buffer(buffer, buffer_length,
                                           copy_start, one_string_length,
                                           byte_table + i * size_of_one_line_in_table);

            received_length_array[i] = one_string_length;

            total_copied_size += one_string_length;

        }

        if (number_of_code < code_array_length) {

            one_string_length =
                calculate_circular_length_in_circular_buffer(buffer, buffer_length,
                                                             *past_copied_end_out,
                                                             code_array[0]) + code_length;

            copy_start = *past_copied_end_out;

            copy_byte_from_circular_buffer(buffer, buffer_length,
                                           copy_start, one_string_length,
                                           byte_table + (number_of_code - 1) * size_of_one_line_in_table);

            received_length_array[number_of_code - 1] = one_string_length;

        }

        *past_copied_end_out =
            calculate_shifted_constant_pointer_in_circular_buffer(buffer, buffer_length,
                                                                  code_array[number_of_code - 1], code_length);

    }

    return total_copied_size;
}

void copy_byte_to_string(const char *buffer, unsigned int buffer_length,
                         std::string &destination)
{
    if (destination.capacity() < destination.size() + buffer_length) {
        destination.reserve(buffer_length + destination.size());
    }

    for (unsigned int i = 0; i < buffer_length; ++i) {
        destination.push_back(buffer[i]);
    }
    return;
}

unsigned int parse_separated_byte_array(const char *buffer, unsigned int buffer_length,
                                        char separator,
                                        std::vector<std::string> &parsed_string_array)
{
    if (parsed_string_array.size() != 0) {
        parsed_string_array.clear();
    }

    std::string temp_string;

    for (unsigned int i = 0; i < buffer_length; ++i) {

        if (buffer[i] == separator) {

            parsed_string_array.push_back(temp_string);
            temp_string.clear();

        } else {
            temp_string.push_back(buffer[i]);
        }

    }

    if (temp_string.size() != 0) {
        parsed_string_array.push_back(temp_string);
    }

    return parsed_string_array.size();
}

void initialize_circular_buffer(circular_buffer_t *buffer)
{

    buffer->length_byte = 0;

    buffer->data_start_point = NULL;
    buffer->destination_point = NULL;
    buffer->used_data_end_out_point = NULL;

    buffer->empty_buffer_length_byte = 0;

    return;
}

void clear_memory_of_circular_buffer(circular_buffer_t *buffer)
{
    memset((void *)buffer->data_start_point, 0, buffer->length_byte);

    buffer->destination_point = buffer->data_start_point;
    buffer->used_data_end_out_point = buffer->data_start_point;

    buffer->empty_buffer_length_byte = buffer->length_byte;

    return;
}

bool allocate_memory_for_circular_buffer(circular_buffer_t *buffer, unsigned int buffer_length_byte)
{

    //! check the buffer is initialized
    if ((buffer->length_byte != 0) ||
        (buffer->data_start_point != NULL) ||
        (buffer->destination_point != NULL) ||
        (buffer->used_data_end_out_point != NULL)) {

        return false;
    }

    //! when the buffer length is zero return false
    if (buffer_length_byte == 0) {

        return false;
    }

    buffer->data_start_point =
        (char *)malloc(buffer_length_byte * sizeof(char));

    if (buffer->data_start_point == NULL) {

        return false;
    }

    buffer->length_byte = buffer_length_byte;

    clear_memory_of_circular_buffer(buffer);

    return true;
}

bool release_memory_of_circular_buffer(circular_buffer_t *buffer)
{

    if ((buffer->data_start_point == NULL) ||
        (buffer->length_byte == 0)) {

        return false;
    }

    free(buffer->data_start_point);

    initialize_circular_buffer(buffer);

    return true;
}

unsigned int get_length_of_empty_data_buffer(const circular_buffer_t *buffer)
{
    return buffer->empty_buffer_length_byte;
}

unsigned int copy_byte_to_circular_buffer(const char *source, unsigned int source_length,
                                          circular_buffer_t *buffer)
{
    unsigned int copied_data_byte = 0;

    if (source_length > buffer->empty_buffer_length_byte) {
        copied_data_byte = buffer->empty_buffer_length_byte;
    } else {
        copied_data_byte = source_length;
    }

    copy_byte_to_circular_buffer(source, copied_data_byte,
                                 buffer->data_start_point, buffer->length_byte,
                                 buffer->destination_point);

    buffer->destination_point =
        calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                                     buffer->destination_point, copied_data_byte);

    buffer->empty_buffer_length_byte = buffer->empty_buffer_length_byte - copied_data_byte;

    return copied_data_byte;
}

unsigned int get_data_byte_length_to_copy_to_circular_buffer(enum ENUMERATION_FOR_BYTE_COPY_MODE copy_mode,
                                                             const circular_buffer_t *buffer,
                                                             unsigned int source_length)
{
    unsigned int copied_data_byte = 0;

    if (source_length > buffer->empty_buffer_length_byte) {

        switch (copy_mode) {

            case BYTE_COPY_MODE_OVERWRITE:
                copied_data_byte = source_length;
                break;

            case BYTE_COPY_MODE_OVERWRITE_PROHIBIT_PARTIAL_WRITE:
                copied_data_byte = source_length;
                break;

            case BYTE_COPY_MODE_POSSIBLE_ALL_WRITE:
                copied_data_byte = buffer->empty_buffer_length_byte;
                break;

            default:
                return 0;
                break;

        }

        if (copy_mode == BYTE_COPY_MODE_OVERWRITE_PROHIBIT_PARTIAL_WRITE) {

            if (copied_data_byte > buffer->length_byte) {
                return 0;
            }

        }

    } else {

        copied_data_byte = source_length;

    }

    return copied_data_byte;
}

unsigned int copy_byte_to_circular_buffer(const char *source, unsigned int source_length,
                                          circular_buffer_t *buffer, enum ENUMERATION_FOR_BYTE_COPY_MODE copy_mode)
{
    const unsigned int copied_data_byte =
        get_data_byte_length_to_copy_to_circular_buffer(copy_mode, buffer,
                                                        source_length);

    copy_byte_to_circular_buffer(source, copied_data_byte,
                                 buffer->data_start_point, buffer->length_byte,
                                 buffer->destination_point);

    buffer->destination_point =
        calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                                     buffer->destination_point, copied_data_byte);


    if (copied_data_byte > buffer->empty_buffer_length_byte) {

        buffer->empty_buffer_length_byte = 0;
        buffer->used_data_end_out_point = buffer->destination_point;

    } else {

        buffer->empty_buffer_length_byte = buffer->empty_buffer_length_byte - copied_data_byte;

    }

    return copied_data_byte;
}

unsigned int calculate_remaining_data_length(const circular_buffer_t *buffer, const char *start)
{
    if (buffer->empty_buffer_length_byte == buffer->length_byte) {
        return 0;
    }

    if (buffer->empty_buffer_length_byte != 0) {

        bool is_remaining_data_pointer =
            is_pointer_in_interval_of_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                                      buffer->used_data_end_out_point, buffer->destination_point,
                                                      start);

        if (is_remaining_data_pointer == false) {
            return 0;
        }

    }

    unsigned int remaining_data_length = 0;

    if (start > buffer->destination_point) {

        const char *buffer_end_out = buffer->data_start_point + buffer->length_byte;

        remaining_data_length =
            buffer_end_out - start +
            buffer->destination_point - buffer->data_start_point;

    } else if (start == buffer->destination_point) {

        if (buffer->empty_buffer_length_byte == 0) {
            remaining_data_length = buffer->length_byte;
        } else {
            remaining_data_length = 0;
        }

    } else {
        remaining_data_length = buffer->destination_point - start;
    }

    return remaining_data_length;
}

unsigned int copy_byte_from_circular_buffer(const circular_buffer_t *buffer,
                                            const char *source, unsigned int source_length,
                                            char *destination, enum ENUMERATION_FOR_BYTE_COPY_MODE copy_mode)
{
    const unsigned int remaining_data_length =
        calculate_remaining_data_length(buffer, source);

    unsigned int actual_copy_length = 0;

    if (remaining_data_length < source_length) {

        switch (copy_mode) {

            case BYTE_COPY_MODE_PROHIBIT_PARTIAL_READ:
                actual_copy_length = 0;
                break;

            case BYTE_COPY_MODE_POSSIBLE_ALL_READ:
                actual_copy_length = remaining_data_length;
                break;

            default:
                actual_copy_length = 0;
                break;
        }

    } else {

        switch (copy_mode) {

            case BYTE_COPY_MODE_PROHIBIT_PARTIAL_READ:
                actual_copy_length = source_length;
                break;

            case BYTE_COPY_MODE_POSSIBLE_ALL_READ:
                actual_copy_length = source_length;
                break;

            default:
                actual_copy_length = 0;
                break;
        }

    }

    if (actual_copy_length == 0) {
        return 0;
    }

    copy_byte_from_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                   source, actual_copy_length,
                                   destination);

    return actual_copy_length;

}

unsigned int calculate_remaining_data_length(const circular_buffer_t *buffer)
{

    unsigned int remaining_data_length = buffer->length_byte - buffer->empty_buffer_length_byte;

    if (buffer->length_byte < buffer->empty_buffer_length_byte) {
        return 0;
    }

    return remaining_data_length;

}

void move_used_data_end_out_point(circular_buffer_t *buffer, unsigned int move_amount)
{

    buffer->used_data_end_out_point =
        calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                                     buffer->used_data_end_out_point, move_amount);

    if (buffer->empty_buffer_length_byte + move_amount > buffer->length_byte) {
        buffer->empty_buffer_length_byte = buffer->length_byte;
    } else {
        buffer->empty_buffer_length_byte += move_amount;
    }
    return;
}

unsigned int move_used_data_end_out_point(circular_buffer_t *buffer, const char *used_data_end_out)
{
    if (buffer->empty_buffer_length_byte == buffer->length_byte) {
        return 0;
    }

    if (buffer->empty_buffer_length_byte != 0) {

        const char *move_limit =
            calculate_shifted_pointer_in_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                                         buffer->destination_point, 1);

        bool is_remaining_data_pointer =
            is_pointer_in_interval_of_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                                      buffer->used_data_end_out_point, move_limit,
                                                      used_data_end_out);

        if (is_remaining_data_pointer == false) {
            return 0;
        }

    }

    unsigned int omitting_byte_size = 0;
    if (buffer->used_data_end_out_point < used_data_end_out) {
        omitting_byte_size = used_data_end_out - buffer->used_data_end_out_point;
    } else {
        const char *buffer_end_out = buffer->data_start_point + buffer->length_byte;

        omitting_byte_size =
            buffer_end_out - buffer->used_data_end_out_point +
            used_data_end_out - buffer->data_start_point;
    }

    move_used_data_end_out_point(buffer, omitting_byte_size);

    return omitting_byte_size;
}

unsigned int copy_non_used_data_byte_from_circular_buffer(circular_buffer_t *source, unsigned int source_length,
                                                          char *destination, unsigned int destination_length)
{
    unsigned int copied_length = source_length;
    if (copied_length > destination_length) {
        copied_length = destination_length;
    }

    const char *copy_start = source->used_data_end_out_point;

    copy_byte_from_circular_buffer(source->data_start_point, source->length_byte,
                                   copy_start, copied_length,
                                   destination);

    move_used_data_end_out_point(source, copied_length);

    return copied_length;


}

unsigned int copy_first_tailed_byte_array_from_circular_buffer(circular_buffer_t *buffer,
                                                               const char *code, unsigned int code_length,
                                                               char *destination_buffer, unsigned int destination_buffer_length,
                                                               unsigned int *tailed_byte_array_length)
{
    unsigned int copied_length = 0;
    *tailed_byte_array_length = 0;

    const unsigned int remaining_data_length = calculate_remaining_data_length(buffer);
    if (remaining_data_length < code_length) {
        return copied_length;
    }

    char *seek_start = buffer->used_data_end_out_point;
    char *seek_end_out = buffer->destination_point;

    const char *first_code_point =
        seek_first_code_from_circular_byte_array(code, code_length,
                                                 buffer->data_start_point, buffer->length_byte,
                                                 seek_start, seek_end_out);

    if (first_code_point == NULL) {
        return copied_length;
    }

    if (first_code_point == seek_start) {
        *tailed_byte_array_length = code_length;
    } else {
        *tailed_byte_array_length = calculate_circular_length_in_circular_buffer(buffer->data_start_point, buffer->length_byte,
                                                                                 seek_start, first_code_point) + code_length;
    }

    copied_length = copy_non_used_data_byte_from_circular_buffer(buffer, *tailed_byte_array_length,
                                                                 destination_buffer, destination_buffer_length);

    return copied_length;
}

void shift_byte_data(char *buffer, unsigned int buffer_length,
                     int shift_amount, char fill_data)
{
    if (buffer_length == 0) {
        return;
    }

    if (shift_amount > 0) {

        if ((int)buffer_length <= shift_amount) {

            memset((void *)buffer, fill_data, buffer_length);

        } else {

            for (int i = ((int)buffer_length) - 1; i >= shift_amount; --i) {
                buffer[i] = buffer[i - shift_amount];
            }

            for (unsigned int i = 0; i < (unsigned int)shift_amount; ++i) {
                buffer[i] = fill_data;
            }

        }

    } else {

        if ((int)buffer_length <= -shift_amount) {

            memset((void *)buffer, fill_data, buffer_length);

        } else {

            for (int i = 0; i < ((int)buffer_length) + shift_amount; ++i) {
                buffer[i] = buffer[i - shift_amount];
            }

            for (int i = ((int)buffer_length) + shift_amount; i < (int)buffer_length; ++i) {
                buffer[i] = fill_data;
            }
        }
    }

    return;
}

unsigned int seek_template_forward_match(const char *template_array, unsigned int template_length,
                                         const char *data_array, unsigned int data_length)
{
    unsigned int seek_length = template_length;

    if (seek_length > data_length) {
        seek_length = data_length;
    }

    unsigned int *match_count_array = NULL;
    match_count_array = (unsigned int *)malloc(seek_length * sizeof(unsigned int));

    if (match_count_array == NULL) {
        return 0;
    }

    for (unsigned int i = 0; i < seek_length; ++i) {

        match_count_array[i] = 0;

        if (template_array[i] == data_array[data_length - 1]) {

            match_count_array[i] = 1;
            for (unsigned int j = 0; j < i; ++j) {

                match_count_array[i] = match_count_array[i] + 1;
                if (template_array[i - j - 1] != data_array[data_length - 2 - j]) {
                    match_count_array[i] = 0;
                    break;
                }

            }
        }

    }

    unsigned int maximum_match_count = 0;

    for (unsigned int i = 0; i < seek_length; ++i) {

        if (match_count_array[i] == i + 1) {
            maximum_match_count = i + 1;
        }

    }

    free(match_count_array);

    return maximum_match_count;
}

unsigned int shift_forward_matching_byte_data(char *data, unsigned int data_length, char fill_data,
                                              const char *template_array, unsigned int template_length)
{

    const unsigned int match_count =
        seek_template_forward_match(template_array, template_length,
                                    data, data_length);

    const int shift_amount = -(data_length - match_count);

    shift_byte_data(data, data_length,
                    shift_amount, fill_data);

    return match_count;
}
