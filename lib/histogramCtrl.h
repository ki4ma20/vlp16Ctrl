#ifndef HISTOGRAM_CONTROL_H
#define HISTOGRAM_CONTROL_H
/*!
  \file
  \brief functions to control histogram
  \author Kiyoshi MATSUO
  $Id$
*/

#include <vector>

#include <ostream>

//! structure for parameters of histogram
struct histogram_t {

    //! minimum value
    double minimum_value;

    //! maximum value
    double maximum_value;

    //! data count
    std::vector<int> data_count;

};

/*!
  \brief function to set parameters of histogram
*/
extern void set_parameters_of_histogram(histogram_t *histogram,
                                        double minimum_value, double maximum_value,
                                        unsigned int number_of_bin);

/*!
  \brief function to set parameters of histogram (step value indicate)
*/
extern void set_parameters_of_histogram(histogram_t *histogram,
                                        double minimum_value, double maximum_value,
                                        double one_step_value);

/*!
  \brief function to convert histogram
  \attention this function uses original one_step_value
*/
extern void change_range_of_histogram(const histogram_t *histogram,
                                      double minimum_value, double maximum_value,
                                      histogram_t *range_changed_histogram);

/*!
  \brief function to copy parameters of histogram
*/
extern void copy_parameters_of_histogram(histogram_t *destination,
                                         const histogram_t *source);

/*!
  \brief function to clear data count of histogram
*/
extern void clear_data_count_of_histogram(histogram_t *histogram);

/*!
  \brief function to make histogram from data array
*/
extern void make_histogram(histogram_t *histogram, const std::vector<double> &data_array);

/*!
  \brief function to make histogram from integer data array
*/
extern void make_histogram(histogram_t *histogram, const std::vector<int> &integer_data_array,
                           std::vector<int> &small_integer_index_array,
                           std::vector<int> &large_integer_index_array);

/*!
  \brief function to get non zero bin values
  \attention this function set boundary zero bin value
*/
extern void get_large_bin_values_of_histogram(const histogram_t *histogram,
                                              int small_bin_threshold,
                                              std::vector<double> &non_zero_bin_values,
                                              std::vector<int> &non_zero_bin_counts);

/*!
  \brief function to get non zero bin values
  \attention this function set boundary zero bin value
*/
extern void get_large_bin_values_of_histogram(const histogram_t *histogram,
                                              int small_bin_threshold,
                                              std::vector<double> &non_zero_bin_values,
                                              std::vector<int> &non_zero_bin_counts,
                                              int *total_count_of_small_bin_count);

/*!
  /brief function to get minimum and maximum data from data array
*/
extern void seek_minimum_and_maximum(int *minimum_index, double *minimum_value,
                                     int *maximum_index, double *maximum_value,
                                     const std::vector<double> &data_array);

/*!
  \brief function to make width adjusted histogram from data array
*/
extern void make_width_adjusted_histogram(histogram_t *histogram, unsigned int number_of_bin,
                                          const std::vector<double> &data_array);

/*!
  \brief function to output histogram to stream
*/
extern void output_histogram_to_stream(std::ostream &stream, const histogram_t *histogram);

/*!
  \brief function to calculate the integral of histogram
*/
extern double calculate_the_integral_of_histogram(const histogram_t *histogram);

/*!
  \brief function to calculate step width of histogram
*/
extern double calculate_step_width_of_histogram(const histogram_t *histogram);

/*!
  \brief function to calculate step width of histogram
*/
extern void calculate_value_array_of_bin_of_histogram(const histogram_t *histogram,
                                                      std::vector<double> &value_array);

/*!
  \brief function to output integral-normalized histogram to stream
*/
extern void output_integral_normalized_histogram_to_stream(std::ostream &stream, const histogram_t *histogram);

/*!
  \brief function to calculate region adjusted histogram
*/
extern void calculate_region_adjusted_histogram(histogram_t *histogram, double one_step_value,
                                                const std::vector<double> &value_array,
                                                double *minimum_value, double *maximum_value,
                                                double *average_value, double *variance_value);

//! structure for statistical information of value array
struct histogram_and_statistics_t {

    //! histogram
    histogram_t histogram;

    //! threshold for small bin
    int small_bin_threshold;

    //! value array of large bin
    std::vector<double> large_bin_value_array;
    //! counts of large bin value
    std::vector<int> large_bin_count_array;

    //! total counts of small bin count
    int total_count_of_small_bin_count;

    //! total number of data
    unsigned int total_number_of_data;

    //! average
    double average;
    //! variance
    double variance;

    //! minimum
    double minimum;
    //! maximum
    double maximum;

};

/*!
  \brief function to calculate region adjusted histograms
  \attention this function does not work if size of each value_array are not equal.
*/
extern void calculate_region_adjusted_histogram_and_statistics_array(const std::vector<double> &one_step_value_array,
                                                                     const std::vector<int> &small_bin_threshold_array,
                                                                     const std::vector< std::vector<double> > &value_table,
                                                                     std::vector<histogram_and_statistics_t *> &histogram_and_statistics_array);

//! structure for parameters of two dimensional histogram
struct two_dimensional_histogram_t {

    //! minimum value1
    double minimum_value1;

    //! maximum value1
    double maximum_value1;

    //! number of bin1
    int number_of_bin1;

    //! minimum value2
    double minimum_value2;

    //! maximum value2
    double maximum_value2;

    //! number of bin2
    int number_of_bin2;

    //! data count
    std::vector<int> data_count;

};

/*!
  \brief function to set parameters of histogram
*/
extern void set_parameters_of_histogram(two_dimensional_histogram_t *histogram,
                                        double minimum_value1, double maximum_value1,
                                        int number_of_bin1,
                                        double minimum_value2, double maximum_value2,
                                        int number_of_bin2);

/*!
  \brief function to set parameters of histogram
*/
extern void set_parameters_of_histogram_by_step_value(two_dimensional_histogram_t *histogram,
                                                      double minimum_value1, double maximum_value1,
                                                      double one_step_value1,
                                                      double minimum_value2, double maximum_value2,
                                                      double one_step_value2);

/*!
  \brief function to copy parameters of histogram
*/
extern void copy_parameters_of_histogram(two_dimensional_histogram_t *destination,
                                         const two_dimensional_histogram_t *source);

/*!
  \brief function to clear data count of histogram
*/
extern void clear_data_count_of_histogram(two_dimensional_histogram_t *histogram);

/*!
  \brief function to make histogram from data array
*/
extern void make_histogram(two_dimensional_histogram_t *histogram,
                           const std::vector<double> &data_array1,
                           const std::vector<double> &data_array2);

/*!
  \brief function to make histogram from data array
*/
extern void make_histogram(two_dimensional_histogram_t *histogram,
                           const std::vector<int> &data_array1,
                           const std::vector<int> &data_array2);

/*!
  \brief function to make width adjusted histogram from data array
*/
extern void make_width_adjusted_histogram(two_dimensional_histogram_t *histogram,
                                          unsigned int number_of_bin1,
                                          unsigned int number_of_bin2,
                                          const std::vector<double> &data_array1,
                                          const std::vector<double> &data_array2);

/*!
  \brief function to output histogram to stream
*/
extern void output_histogram_to_stream(std::ostream &stream, const two_dimensional_histogram_t *histogram);

/*!
  \brief function to get large bin values of histogram
 */
extern void get_large_bin_values_of_histogram(const two_dimensional_histogram_t *histogram,
                                              int small_bin_threshold,
                                              std::vector<double> &large_bin_value1,
                                              std::vector<double> &large_bin_value2,
                                              std::vector<int> &large_bin_counts);


#endif // HISTOGRAM_CONTROL_H
