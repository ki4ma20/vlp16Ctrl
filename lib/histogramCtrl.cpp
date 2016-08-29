

#include "histogramCtrl.h"

void set_parameters_of_histogram(histogram_t *histogram,
                                 double minimum_value, double maximum_value,
                                 unsigned int number_of_bin)
{
    histogram->minimum_value = minimum_value;
    histogram->maximum_value = maximum_value;

    if (histogram->data_count.size() != 0) {
        histogram->data_count.clear();
    }
    histogram->data_count.resize(number_of_bin, 0);

    return;
}

void set_parameters_of_histogram(histogram_t *histogram,
                                 double minimum_value, double maximum_value,
                                 double one_step_value)
{
    if (one_step_value <= 0.0) {
        return;
    }
    const double number_of_steps =
        (maximum_value - minimum_value) / one_step_value;

    const unsigned int number_of_bin =
        (unsigned int)(number_of_steps) + 1;

    set_parameters_of_histogram(histogram, minimum_value, maximum_value,
                                number_of_bin);

    return;
}

void change_range_of_histogram(const histogram_t *histogram,
                               double minimum_value, double maximum_value,
                               histogram_t *range_changed_histogram)
{
    double step_width = histogram->maximum_value - histogram->minimum_value;
    if (histogram->data_count.size() > 1) {
        step_width =
            (histogram->maximum_value - histogram->minimum_value) / (double)(histogram->data_count.size() - 1);
    }

    set_parameters_of_histogram(range_changed_histogram,
                                minimum_value, maximum_value,
                                step_width);

    if (step_width == 0.0) {
        range_changed_histogram->data_count.resize(1);
    }

    clear_data_count_of_histogram(range_changed_histogram);

    const double step_coefficient =
        (double)(range_changed_histogram->data_count.size() - 1) /
        (maximum_value - minimum_value);

    double bin = histogram->minimum_value;
    double data = bin;
    unsigned int quantized_data = 0;

    for (int i = 0; i < (int)histogram->data_count.size(); ++i) {

        if (bin <= minimum_value) {
            data = minimum_value;
        } else if (bin < maximum_value) {
            data = bin;
        } else  {
            data = maximum_value;
        }

        data -=minimum_value;
        quantized_data = (unsigned int) (data * step_coefficient);
        range_changed_histogram->data_count.at(quantized_data) += histogram->data_count.at(i);

        bin += step_width;
    }

    return;
}

void copy_parameters_of_histogram(histogram_t *destination,
                                  const histogram_t *source)
{

    destination->minimum_value = source->minimum_value;
    destination->maximum_value = source->maximum_value;

    return;
}

void clear_data_count_of_histogram(histogram_t *histogram)
{

    for (unsigned int i = 0; i < histogram->data_count.size(); ++i) {
        histogram->data_count.at(i) = 0;
    }

    return;
}

void make_histogram(histogram_t *histogram, const std::vector<double> &data_array)
{

    clear_data_count_of_histogram(histogram);

    double step_coefficient = 0.0;

    if (histogram->maximum_value != histogram->minimum_value) {
        step_coefficient =
            (double)(histogram->data_count.size() - 1) /
            (histogram->maximum_value - histogram->minimum_value);
    }

    unsigned int quantized_value = 0;
    double data = 0.0;

    for (int i = 0; i < (int)data_array.size(); ++i) {

        data = data_array.at(i);

        if (data < histogram->minimum_value) {

            data = histogram->minimum_value;

        } else if (data > histogram->maximum_value) {

            data = histogram->maximum_value;

        }
        data -= histogram->minimum_value;
        quantized_value = (unsigned int) (data * step_coefficient);

        if (histogram->data_count.size() > quantized_value) {
            ++histogram->data_count.at(quantized_value);
        }

    }

    return;
}

void make_histogram(histogram_t *histogram, const std::vector<int> &data_array,
                    std::vector<int> &small_integer_index_array,
                    std::vector<int> &large_integer_index_array)
{

    clear_data_count_of_histogram(histogram);

    double step_coefficient = 0.0;

    if (histogram->maximum_value != histogram->minimum_value) {
        step_coefficient =
            (double)(histogram->data_count.size() - 1) /
            (histogram->maximum_value - histogram->minimum_value);
    }

    unsigned int quantized_value = 0;

    double data = histogram->minimum_value;

    for (int i = 0; i < (int)data_array.size(); ++i) {

        data = (double)data_array.at(i);

        if (data < histogram->minimum_value) {

            small_integer_index_array.push_back(i);

        } else if (data > histogram->maximum_value) {

            large_integer_index_array.push_back(i);

        }

        data -= histogram->minimum_value;
        quantized_value = (unsigned int) (data * step_coefficient);

        if (histogram->data_count.size() > quantized_value) {
            ++histogram->data_count.at(quantized_value);
        }

    }

    return;
}

void get_large_bin_values_of_histogram(const histogram_t *histogram,
                                       int small_bin_threshold,
                                       std::vector<double> &non_zero_bin_values,
                                       std::vector<int> &non_zero_bin_counts)
{
    double step_coefficient = 0.0;

    if (histogram->maximum_value != histogram->minimum_value) {
        step_coefficient =
            (histogram->maximum_value - histogram->minimum_value) / (double)(histogram->data_count.size() - 1);
    }

    const std::vector<int> *data_count = &histogram->data_count;

    for (unsigned int bin_index = 0; bin_index < data_count->size(); ++bin_index) {

        if (data_count->at(bin_index) > small_bin_threshold) {

            if (bin_index > 0) {

                if (data_count->at(bin_index - 1) <= small_bin_threshold) {

                    double past_value =
                        (((double)bin_index - 1) + 0.5) * step_coefficient + histogram->minimum_value;

                    if (non_zero_bin_values.size() == 0) {

                        non_zero_bin_values.push_back(past_value);
                        non_zero_bin_counts.push_back(data_count->at(bin_index - 1));


                    } else {

                        if (non_zero_bin_values.at(non_zero_bin_values.size() - 1) != past_value) {

                            non_zero_bin_values.push_back(past_value);
                            non_zero_bin_counts.push_back(data_count->at(bin_index - 1));

                        }

                    }

                }


            }


            double value = (((double)bin_index) + 0.5) * step_coefficient + histogram->minimum_value;

            non_zero_bin_values.push_back(value);
            non_zero_bin_counts.push_back(data_count->at(bin_index));

            if (bin_index + 1 < data_count->size()) {

                if (data_count->at(bin_index + 1) <= small_bin_threshold) {

                    double next_value =
                        (((double)bin_index + 1) + 0.5) * step_coefficient + histogram->minimum_value;

                    non_zero_bin_values.push_back(next_value);
                    non_zero_bin_counts.push_back(data_count->at(bin_index + 1));

                }

            }

        }


    }

    return;
}

void get_large_bin_values_of_histogram(const histogram_t *histogram,
                                       int small_bin_threshold,
                                       std::vector<double> &non_zero_bin_values,
                                       std::vector<int> &non_zero_bin_counts,
                                       int *total_count_of_small_bin_count)
{
    double step_coefficient = 0.0;

    if (histogram->maximum_value != histogram->minimum_value) {
        step_coefficient =
            (histogram->maximum_value - histogram->minimum_value) / (double)(histogram->data_count.size() - 1);

        if (histogram->data_count.size() == 1) {
            step_coefficient = 1.0;
        }

    }

    const std::vector<int> *data_count = &histogram->data_count;
    *total_count_of_small_bin_count = 0.0;
    for (unsigned int bin_index = 0; bin_index < data_count->size(); ++bin_index) {

        if (data_count->at(bin_index) > small_bin_threshold) {

            if (bin_index > 0) {

                if (data_count->at(bin_index - 1) <= small_bin_threshold) {

                    double past_value =
                        (((double)bin_index - 1) + 0.5) * step_coefficient + histogram->minimum_value;

                    if (non_zero_bin_values.size() == 0) {

                        non_zero_bin_values.push_back(past_value);
                        non_zero_bin_counts.push_back(data_count->at(bin_index - 1));


                    } else {

                        if (non_zero_bin_values.at(non_zero_bin_values.size() - 1) != past_value) {

                            non_zero_bin_values.push_back(past_value);
                            non_zero_bin_counts.push_back(data_count->at(bin_index - 1));

                        }

                    }

                }


            }


            double value = (((double)bin_index) + 0.5) * step_coefficient + histogram->minimum_value;

            non_zero_bin_values.push_back(value);
            non_zero_bin_counts.push_back(data_count->at(bin_index));

            if (bin_index + 1 < data_count->size()) {

                if (data_count->at(bin_index + 1) <= small_bin_threshold) {

                    double next_value =
                        (((double)bin_index + 1) + 0.5) * step_coefficient + histogram->minimum_value;

                    non_zero_bin_values.push_back(next_value);
                    non_zero_bin_counts.push_back(data_count->at(bin_index + 1));

                }

            }

        } else {
            *total_count_of_small_bin_count =
                *total_count_of_small_bin_count + data_count->at(bin_index);
        }

    }

    return;
}

void seek_minimum_and_maximum(int *minimum_index, double *minimum_value,
                              int *maximum_index, double *maximum_value,
                              const std::vector<double> &data_array)
{

    if (data_array.size() == 0) {
        return;
    }

    *minimum_value = data_array.at(0);
    *minimum_index = 0;

    *maximum_value = data_array.at(0);
    *maximum_index = 0;

    for (int i = 0; i < (int)data_array.size(); ++i) {

        if (data_array.at(i) < *minimum_value) {
            *minimum_value = data_array.at(i);
            *minimum_index = i;
        }

        if (data_array.at(i) > *maximum_value) {
            *maximum_value = data_array.at(i);
            *maximum_index = i;
        }

    }

    return;
}

void make_width_adjusted_histogram(histogram_t *histogram, unsigned int number_of_bin,
                                   const std::vector<double> &data_array)
{

    int minimum_index = 0;
    int maximum_index = 0;

    seek_minimum_and_maximum(&minimum_index, &histogram->minimum_value,
                             &maximum_index, &histogram->maximum_value,
                             data_array);

    if (histogram->data_count.size() != 0) {
        histogram->data_count.clear();
    }
    histogram->data_count.resize(number_of_bin, 0);

    make_histogram(histogram, data_array);

    return;
}

void output_histogram_to_stream(std::ostream &stream, const histogram_t *histogram)
{

    double step_width = histogram->maximum_value - histogram->minimum_value;
    if (histogram->data_count.size() > 1) {
        step_width =
            (histogram->maximum_value - histogram->minimum_value) / (double)(histogram->data_count.size() - 1);
    }

    double bin = histogram->minimum_value;

    for (int i = 0; i < (int)histogram->data_count.size(); ++i) {
        stream << bin << "," << (double)histogram->data_count.at(i) << "\n";
        bin += step_width;
    }

    return;
}

double calculate_the_integral_of_histogram(const histogram_t *histogram)
{
    const double step_width =
        (histogram->maximum_value - histogram->minimum_value) / (double)(histogram->data_count.size() - 1);

    double integral = 0.0;

    for (int i = 0; i < (int)histogram->data_count.size(); ++i) {

        integral += step_width * (double)histogram->data_count.at(i);

    }

    return integral;
}

double calculate_step_width_of_histogram(const histogram_t *histogram)
{
    if (histogram->data_count.size() <= 1) {
        return 0.0;
    }

    const double step_width =
        (histogram->maximum_value - histogram->minimum_value) / (double)(histogram->data_count.size() - 1);

    return step_width;
}

void calculate_value_array_of_bin_of_histogram(const histogram_t *histogram,
                                               std::vector<double> &value_array)
{
    const double step_width = calculate_step_width_of_histogram(histogram);

    if (value_array.size() != 0) {
        value_array.clear();
    }

    value_array.resize(histogram->data_count.size(), 0);
    double bin = histogram->minimum_value;

    for (int i = 0; i < (int)histogram->data_count.size(); ++i) {
        value_array.at(i) = bin;
        bin += step_width;
    }

    return;
}

void output_integral_normalized_histogram_to_stream(std::ostream &stream, const histogram_t *histogram)
{
    const double step_width = calculate_step_width_of_histogram(histogram);
    const double inverse_of_integral = 1.0 / calculate_the_integral_of_histogram(histogram);

    double bin = histogram->minimum_value;

    for (int i = 0; i < (int)histogram->data_count.size(); ++i) {
        stream << bin << "," << inverse_of_integral * (double)histogram->data_count.at(i) << "\n";
        bin += step_width;
    }

    return;
}

void calculate_region_adjusted_histogram(histogram_t *histogram, double one_step_value,
                                         const std::vector<double> &value_array,
                                         double *minimum_value, double *maximum_value,
                                         double *average_value, double *variance_value)
{
    *minimum_value = 0.0;
    *maximum_value = 0.0;

    if (value_array.size() == 0) {
        return;
    }

    const double inverse_of_number_of_values =
        1.0 / (double)value_array.size();

    *average_value = value_array.at(0);

    *minimum_value = value_array.at(0);
    *maximum_value = value_array.at(0);

    for (unsigned int i = 1; i < value_array.size(); ++i) {

        if (value_array.at(i) < *minimum_value) {
            *minimum_value = value_array.at(i);
        }

        if (value_array.at(i) > *maximum_value) {
            *maximum_value = value_array.at(i);
        }

        *average_value = *average_value + value_array.at(i);

    }

    *average_value = inverse_of_number_of_values * (*average_value);

    set_parameters_of_histogram(histogram,
                                *minimum_value, *maximum_value,
                                one_step_value);

    clear_data_count_of_histogram(histogram);

    double step_coefficient = 0.0;

    if (histogram->maximum_value != histogram->minimum_value) {
        step_coefficient =
            (double)(histogram->data_count.size() - 1) /
            (histogram->maximum_value - histogram->minimum_value);
    }

    unsigned int quantized_value = 0;
    double data = 0.0;

    double data_difference = 0.0;
    *variance_value = 0.0;

    for (int i = 0; i < (int)value_array.size(); ++i) {

        data = value_array.at(i);
        data_difference = data - (*average_value);
        *variance_value = (*variance_value) + data_difference * data_difference;

        if (data < histogram->minimum_value) {

            data = histogram->minimum_value;

        } else if (data > histogram->maximum_value) {

            data = histogram->maximum_value;

        }
        data -= histogram->minimum_value;
        quantized_value = (unsigned int) (data * step_coefficient);

        if (histogram->data_count.size() > quantized_value) {
            ++histogram->data_count.at(quantized_value);
        }

    }
    *variance_value = inverse_of_number_of_values * (*variance_value);

    return;
}

static void clear_histogram_and_statistics(histogram_and_statistics_t *histogram)
{
    histogram->small_bin_threshold = 0.0;

    if (histogram->large_bin_value_array.size() != 0) {
        histogram->large_bin_value_array.clear();
    }

    if (histogram->large_bin_count_array.size() != 0) {
        histogram->large_bin_count_array.clear();
    }

    histogram->total_number_of_data = 0;
    histogram->total_count_of_small_bin_count = 0;

    histogram->average = 0.0;
    histogram->variance = 0.0;

    histogram->minimum = 0.0;
    histogram->maximum = 0.0;

    return;
}


void calculate_region_adjusted_histogram_and_statistics_array(const std::vector<double> &one_step_value_array,
                                                              const std::vector<int> &small_bin_threshold_array,
                                                              const std::vector< std::vector<double> > &value_table,
                                                              std::vector<histogram_and_statistics_t *> &histogram_array)
{
    if (value_table.size() == 0) {
        return;
    }

    const unsigned int number_of_data = value_table.at(0).size();

    for (unsigned int i = 0; i < value_table.size(); ++i) {

        if (value_table.at(i).size() != number_of_data) {
            return;
        }

    }

    if (one_step_value_array.size() != value_table.size()) {
        return;
    }
    if (small_bin_threshold_array.size() != value_table.size()) {
        return;
    }

    if (number_of_data == 0) {
        return;
    }

    if (histogram_array.size() != value_table.size()) {
        histogram_array.clear();
        histogram_array.resize(value_table.size());
    }

    const double inverse_of_size = 1.0 / (double)number_of_data;

    double *minimum_value = NULL;
    double *maximum_value = NULL;

    double *average_value = NULL;
    double *variance_value = NULL;

    const std::vector<double> *value_array = NULL;
    histogram_t *histogram = NULL;

    double one_step_value = 0.0;

    for (unsigned int array_index = 0; array_index < value_table.size(); ++array_index) {

        clear_histogram_and_statistics(histogram_array.at(array_index));

        value_array = &value_table.at(array_index);

        minimum_value = &histogram_array.at(array_index)->minimum;
        maximum_value = &histogram_array.at(array_index)->maximum;

        average_value = &histogram_array.at(array_index)->average;
        variance_value = &histogram_array.at(array_index)->variance;

        one_step_value = one_step_value_array.at(array_index);
        histogram_array.at(array_index)->small_bin_threshold = small_bin_threshold_array.at(array_index);

        *average_value = value_array->at(0);

        *minimum_value = value_array->at(0);
        *maximum_value = value_array->at(0);

        for (unsigned int i = 1; i < value_array->size(); ++i) {

            if (value_array->at(i) < *minimum_value) {
                *minimum_value = value_array->at(i);
            }

            if (value_array->at(i) > *maximum_value) {
                *maximum_value = value_array->at(i);
            }

            *average_value = *average_value + value_array->at(i);

        }

        *average_value = inverse_of_size * (*average_value);

        histogram = &histogram_array.at(array_index)->histogram;

        set_parameters_of_histogram(histogram,
                                    *minimum_value, *maximum_value,
                                    one_step_value);

        clear_data_count_of_histogram(histogram);

        double step_coefficient = 0.0;

        if (histogram->maximum_value != histogram->minimum_value) {
            step_coefficient =
                (double)(histogram->data_count.size() - 1) /
                (histogram->maximum_value - histogram->minimum_value);
        }

        unsigned int quantized_value = 0;
        double data = 0.0;

        double data_difference = 0.0;
        *variance_value = 0.0;

        for (int i = 0; i < (int)value_array->size(); ++i) {

            data = value_array->at(i);
            data_difference = data - (*average_value);
            *variance_value = (*variance_value) + data_difference * data_difference;

            data -= histogram->minimum_value;
            quantized_value = (unsigned int) (data * step_coefficient);

            if (histogram->data_count.size() > quantized_value) {
                ++histogram->data_count.at(quantized_value);
            }

        }
        *variance_value = inverse_of_size * (*variance_value);
        get_large_bin_values_of_histogram(histogram,
                                          histogram_array.at(array_index)->small_bin_threshold,
                                          histogram_array.at(array_index)->large_bin_value_array,
                                          histogram_array.at(array_index)->large_bin_count_array,
                                          &histogram_array.at(array_index)->total_count_of_small_bin_count);

        histogram_array.at(array_index)->total_number_of_data = number_of_data;

    }

    return;
}

void set_parameters_of_histogram(two_dimensional_histogram_t *histogram,
                                 double minimum_value1, double maximum_value1,
                                 int number_of_bin1,
                                 double minimum_value2, double maximum_value2,
                                 int number_of_bin2)
{

    histogram->minimum_value1 = minimum_value1;
    histogram->maximum_value1 = maximum_value1;

    histogram->number_of_bin1 = number_of_bin1;

    histogram->minimum_value2 = minimum_value2;
    histogram->maximum_value2 = maximum_value2;

    histogram->number_of_bin2 = number_of_bin2;

    const int number_of_bin = number_of_bin1 * number_of_bin2;

    if (histogram->data_count.size() != 0) {
        histogram->data_count.clear();
    }
    histogram->data_count.resize(number_of_bin, 0);

    return;
}

void set_parameters_of_histogram_by_step_value(two_dimensional_histogram_t *histogram,
                                               double minimum_value1, double maximum_value1,
                                               double one_step_value1,
                                               double minimum_value2, double maximum_value2,
                                               double one_step_value2)
{
    if ((one_step_value1 <= 0.0) ||
        (one_step_value2 <= 0.0)) {
        return;
    }

    const double number_of_steps1 =
        (maximum_value1 - minimum_value1) / one_step_value1;

    const unsigned int number_of_bin1 =
        (unsigned int)(number_of_steps1) + 1;

    const double number_of_steps2 =
        (maximum_value2 - minimum_value2) / one_step_value2;

    const unsigned int number_of_bin2 =
        (unsigned int)(number_of_steps2) + 1;

    set_parameters_of_histogram(histogram,
                                minimum_value1, maximum_value1,
                                (int)number_of_bin1,
                                minimum_value2, maximum_value2,
                                (int)number_of_bin2);

    return;
}

void copy_parameters_of_histogram(two_dimensional_histogram_t *destination,
                                  const two_dimensional_histogram_t *source)
{

    destination->minimum_value1 = source->minimum_value1;
    destination->maximum_value1 = source->maximum_value1;

    destination->number_of_bin1 = source->number_of_bin1;

    destination->minimum_value2 = source->minimum_value2;
    destination->maximum_value2 = source->maximum_value2;

    destination->number_of_bin2 = source->number_of_bin2;

    return;
}

void clear_data_count_of_histogram(two_dimensional_histogram_t *histogram)
{

    for (unsigned int i = 0; i < histogram->data_count.size(); ++i) {
        histogram->data_count.at(i) = 0;
    }

    return;
}

void make_histogram(two_dimensional_histogram_t *histogram,
                    const std::vector<double> &data_array1,
                    const std::vector<double> &data_array2)
{

    clear_data_count_of_histogram(histogram);
    if (data_array1.size() != data_array2.size()) {
        return;
    }

    if (histogram->number_of_bin1 * histogram->number_of_bin2 != (int)histogram->data_count.size()) {
        return;
    }

    const double step_coefficient1 =
        ((double)histogram->number_of_bin1 - 1) /
        (histogram->maximum_value1 - histogram->minimum_value1);

    const double step_coefficient2 =
        ((double)histogram->number_of_bin2 - 1) /
        (histogram->maximum_value2 - histogram->minimum_value2);

    unsigned int quantized_value1 = 0;
    unsigned int quantized_value2 = 0;

    double data1 = 0.0;
    double data2 = 0.0;

    unsigned int data_index = 0;

    for (int i = 0; i < (int)data_array1.size(); ++i) {

        data1 = data_array1.at(i);
        if (data1 < histogram->minimum_value1) {

            data1 = histogram->minimum_value1;

            continue;

        } else if (data1 > histogram->maximum_value1) {

            data1 = histogram->maximum_value1;

            continue;
        }
        data1 -= histogram->minimum_value1;

        data2 = data_array2.at(i);
        if (data2 < histogram->minimum_value2) {

            data2 = histogram->minimum_value2;
            continue;

        } else if (data2 > histogram->maximum_value2) {

            data2 = histogram->maximum_value2;
            continue;

        }
        data2 -= histogram->minimum_value2;

        quantized_value1 = (unsigned int) (data1 * step_coefficient1);
        quantized_value2 = (unsigned int) (data2 * step_coefficient2);

        data_index = quantized_value2 * histogram->number_of_bin1 + quantized_value1;

        ++histogram->data_count.at(data_index);

    }

    return;
}

void make_histogram(two_dimensional_histogram_t *histogram,
                    const std::vector<int> &data_array1,
                    const std::vector<int> &data_array2)
{

    clear_data_count_of_histogram(histogram);
    if (data_array1.size() != data_array2.size()) {
        return;
    }

    if (histogram->number_of_bin1 * histogram->number_of_bin2 != (int)histogram->data_count.size()) {
        return;
    }

    const double step_coefficient1 =
        ((double)histogram->number_of_bin1 - 1) /
        (histogram->maximum_value1 - histogram->minimum_value1);

    const double step_coefficient2 =
        ((double)histogram->number_of_bin2 - 1) /
        (histogram->maximum_value2 - histogram->minimum_value2);

    unsigned int quantized_value1 = 0;
    unsigned int quantized_value2 = 0;

    double data1 = 0.0;
    double data2 = 0.0;

    unsigned int data_index = 0;

    for (int i = 0; i < (int)data_array1.size(); ++i) {

        data1 = (double)data_array1.at(i);
        if (data1 < histogram->minimum_value1) {

            data1 = histogram->minimum_value1;

            continue;

        } else if (data1 > histogram->maximum_value1) {

            data1 = histogram->maximum_value1;

            continue;
        }
        data1 -= histogram->minimum_value1;

        data2 = (double)data_array2.at(i);
        if (data2 < histogram->minimum_value2) {

            data2 = histogram->minimum_value2;
            continue;

        } else if (data2 > histogram->maximum_value2) {

            data2 = histogram->maximum_value2;
            continue;

        }
        data2 -= histogram->minimum_value2;

        quantized_value1 = (unsigned int) (data1 * step_coefficient1);
        quantized_value2 = (unsigned int) (data2 * step_coefficient2);

        data_index = quantized_value2 * histogram->number_of_bin1 + quantized_value1;

        ++histogram->data_count.at(data_index);

    }

    return;
}

void make_width_adjusted_histogram(two_dimensional_histogram_t *histogram,
                                   unsigned int number_of_bin1,
                                   unsigned int number_of_bin2,
                                   const std::vector<double> &data_array1,
                                   const std::vector<double> &data_array2)
{

    int minimum_index = 0;
    int maximum_index = 0;

    seek_minimum_and_maximum(&minimum_index, &histogram->minimum_value1,
                             &maximum_index, &histogram->maximum_value1,
                             data_array1);

    seek_minimum_and_maximum(&minimum_index, &histogram->minimum_value2,
                             &maximum_index, &histogram->maximum_value2,
                             data_array2);

    histogram->number_of_bin1 = number_of_bin1;
    histogram->number_of_bin2 = number_of_bin2;

    const int number_of_bin = number_of_bin1 * number_of_bin2;

    if (histogram->data_count.size() != 0) {
        histogram->data_count.clear();
    }
    histogram->data_count.resize(number_of_bin, 0);

    make_histogram(histogram, data_array1, data_array2);

    return;
}

void output_histogram_to_stream(std::ostream &stream, const two_dimensional_histogram_t *histogram)
{

    const double step_width1 =
        (histogram->maximum_value1 - histogram->minimum_value1) / (double)(histogram->number_of_bin1 - 1);

    const double step_width2 =
        (histogram->maximum_value2 - histogram->minimum_value2) / (double)(histogram->number_of_bin2 - 1);

    double bin1 = histogram->minimum_value1;
    double bin2 = histogram->minimum_value2;

    for (int j = 0; j < histogram->number_of_bin2; ++j) {
        bin1 = histogram->minimum_value1;
        for (int i = 0; i < histogram->number_of_bin1; ++i) {

            stream << bin1 << "," << bin2 << ","
                   << (double)histogram->data_count.at(j * histogram->number_of_bin1 + i) << "\n";

            bin1 += step_width1;
        }
        bin2 += step_width2;
    }

    return;
}


void get_large_bin_values_of_histogram(const two_dimensional_histogram_t *histogram,
                                       int small_bin_threshold,
                                       std::vector<double> &large_bin_value1,
                                       std::vector<double> &large_bin_value2,
                                       std::vector<int> &large_bin_counts)

{
    const double step_coefficient1 =
        (histogram->maximum_value1 - histogram->minimum_value1) / ((double)histogram->number_of_bin1 - 1);

    const double step_coefficient2 =
        (histogram->maximum_value2 - histogram->minimum_value2) / ((double)histogram->number_of_bin2 - 1);

    const std::vector<int> *data_count = &histogram->data_count;

    for (unsigned int bin_index = 0; bin_index < data_count->size(); ++bin_index) {

        if (data_count->at(bin_index) > small_bin_threshold) {

            const int bin_index1 = bin_index % histogram->number_of_bin1;
            const int bin_index2 = bin_index / histogram->number_of_bin1;

            const double value1 = (((double)bin_index1) + 0.5) * step_coefficient1 + histogram->minimum_value1;
            const double value2 = (((double)bin_index2) + 0.5) * step_coefficient2 + histogram->minimum_value2;

            large_bin_value1.push_back(value1);
            large_bin_value2.push_back(value2);

            large_bin_counts.push_back(data_count->at(bin_index));

        }


    }

    return;
}
