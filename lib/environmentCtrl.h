#ifndef ENVIRONMENT_DEPENDENCE_CONTROL_H
#define ENVIRONMENT_DEPENDENCE_CONTROL_H
/*!
  \file
  \brief header to handle the dependence of environment
  \author Kiyoshi MATSUO
  $Id$
*/

// for ostream
#include <ostream>

#ifdef _WIN32
    #define WINDOWS_OS
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include<windows.h>
    #ifdef _WIN64
        #define BUILDING_X86_64_SYSTEM
    #else
        #define BUILDING_X86_32_SYSTEM
    #endif
#elif __APPLE__
    #include "TargetConditionals.h"
    #define APPLE_OS
    #ifdef TARGET_OS_IPHONE
        #define IPHONE_OS
    #elif TARGET_IPHONE_SIMULATOR
        #define IPHONE_SIMULATOR
    #elif TARGET_OS_MAC
        #define MAC_OS
    #else
        #define APPLE_UNDEFINED_OS
    #endif
#elif __linux__
    #define LINUX_OS
    #ifdef __x86_64__
        #define BUILDING_X86_64_SYSTEM
    #else
        #define BUILDING_X86_32_SYSTEM
    #endif
#else
    #define UNDEFINED_OS
#endif

//! constants for os type
enum OS_TYPE {

    //! windows
    OS_TYPE_WINDOWS,

    //! ios
    OS_TYPE_IOS,

    //! ios simulator
    OS_TYPE_IOS_SIMULATOR,

    //! machintosh
    OS_TYPE_MAC,

    //! undefined apple
    OS_TYPE_UNDEFINED_APPLE,

    //! linux
    OS_TYPE_LINUX,

    //! undefined
    OS_TYPE_UNDEFINED,
};

//! constants for build environment control
enum CONSTANTS_FOR_BUILD_ENVIRONMENT_CONTROL {

    //! maximum length of os version name
    MAXIMUM_LENGTH_OF_OS_VERSION_NAME = 64,

    //! maximum length of compiler name
    MAXIMUM_LENGTH_OF_COMPILER_NAME = 64,

    //! maximum length of compiler version name
    MAXIMUM_LENGTH_OF_COMPILER_VERSION_NAME = 128,

    //! maximum length of architecture name
    MAXIMUM_LENGTH_OF_ARCHITECTURE_NAME = 64,
};

//! structure for environment
struct build_environment_t {
    //! os type
    enum OS_TYPE os;
    //! building bit
    unsigned int binary_building_bit;
    //! os version
    char build_os_version[MAXIMUM_LENGTH_OF_OS_VERSION_NAME];
    //! compiler version
    char compiler_version[MAXIMUM_LENGTH_OF_COMPILER_VERSION_NAME];
    //! executing os version
    char executing_os_version[MAXIMUM_LENGTH_OF_OS_VERSION_NAME];
    //! executing os architecture
    char executing_architecture[MAXIMUM_LENGTH_OF_ARCHITECTURE_NAME];
};

/*!
  \brief function to clear build environment parameter
*/
extern void clear_build_environment_parameter(build_environment_t *environment);

/*!
  \brief function to get build environment
*/
extern void get_build_environment(build_environment_t *environment);

/*!
  \brief function to output build environment to stream
*/
extern void output_build_environment_to_stream(std::ostream &stream, const build_environment_t *environment);

#endif // ENVIRONMENT_DEPENDENCE_CONTROL_H
