
#include "environmentCtrl.h"

// for strcpy
#include <string.h>

// for sprintf
#include <stdio.h>

#ifdef __linux__
// for uname
#include <sys/utsname.h>
#endif

void clear_build_environment_parameter(build_environment_t *environment)
{
    environment->os = OS_TYPE_UNDEFINED;
    environment->binary_building_bit = 0;
    for (unsigned int i = 0; i < MAXIMUM_LENGTH_OF_OS_VERSION_NAME; ++i) {
        environment->build_os_version[i] = 0;
        environment->executing_os_version[i] = 0;
    }

    for (unsigned int i = 0; i < MAXIMUM_LENGTH_OF_COMPILER_VERSION_NAME; ++i) {
        environment->compiler_version[i] = 0;
    }

    for (unsigned int i = 0; i < MAXIMUM_LENGTH_OF_ARCHITECTURE_NAME; ++i) {
        environment->executing_architecture[i] = 0;
    }

    return;
}

static void get_windows_os_name(build_environment_t *environment)
{

#if defined(WINVER) && (WINVER == 0x0500)
        strcpy(environment->build_os_version, "Windows2000");
#elif defined(WINVER) && (WINVER == 0x0501)
        strcpy(environment->build_os_version, "WindowsXP");
#elif defined(WINVER) && (WINVER == 0x0502)
        strcpy(environment->build_os_version, "Windows Server2003");
#elif defined(WINVER) && (WINVER == 0x0600)
        strcpy(environment->build_os_version, "WindowsVista");
#elif defined(WINVER) && (WINVER == 0x0601)
        strcpy(environment->build_os_version, "Windows7");
#elif defined(WINVER) && (WINVER == 0x0602)
        strcpy(environment->build_os_version, "Windows8");
#elif defined(WINVER) && (WINVER == 0x0603)
        strcpy(environment->build_os_version, "Windows8.1");
#else
        strcpy(environment->build_os_version, "Windows Undefined");
#endif

#if defined(WINDOWS_OS)

#if defined(_MSC_VER) && (_MSC_VER >= 1800)
        // use IsWindowsVersionOrGreater to detect Windows 8.1
#else
        OSVERSIONINFO os_information;

        ZeroMemory(&os_information, sizeof(os_information));
        os_information.dwOSVersionInfoSize = sizeof(os_information);

        unsigned int major_version = 0;
        unsigned int minor_version = 0;
        if (GetVersionEx(&os_information) != 0) {
            major_version = (unsigned int)os_information.dwMajorVersion;
            minor_version = (unsigned int)os_information.dwMinorVersion;
        }
        sprintf(environment->executing_os_version,
                "Windows %u.%u", major_version, minor_version);

        const unsigned int xp_major_version = 5;
        const unsigned int xp_minor_version = 1;

        if ((major_version > xp_major_version) ||
            ((major_version == xp_major_version) &&
             (minor_version >= xp_minor_version))) {

            SYSTEM_INFO system_information;
            GetNativeSystemInfo(&system_information);
            switch (system_information.wProcessorArchitecture) {
                case PROCESSOR_ARCHITECTURE_AMD64:
                    strcpy(environment->executing_architecture, "x64");
                    break;

                case PROCESSOR_ARCHITECTURE_IA64:
                    strcpy(environment->executing_architecture, "IA64");
                    break;

                case PROCESSOR_ARCHITECTURE_INTEL:
                    strcpy(environment->executing_architecture, "x86");
                    break;

                case PROCESSOR_ARCHITECTURE_UNKNOWN:
                default:
                    strcpy(environment->executing_architecture, "Undefined architecture");
                    break;

            }
        }

#endif

#endif

    return;
}

static void get_linux_os_name(build_environment_t *environment)
{
#ifdef LINUX_OS
    struct utsname uname_buffer;
    strcpy(environment->build_os_version, "Linux");
    if (uname(&uname_buffer) == 0) {
        strcpy(environment->executing_os_version, uname_buffer.sysname);
        strcat(environment->executing_os_version, " ");
        strcat(environment->executing_os_version, uname_buffer.release);
        strcpy(environment->executing_architecture, uname_buffer.machine);
    } else {
        strcpy(environment->executing_os_version, "Linux Undefined");
        strcpy(environment->executing_architecture, "Undefined architecture");
    }

#endif

#ifdef APPLE_OS
    struct utsname uname_buffer;
    strcpy(environment->build_os_version, "Mac");
    if (uname(&uname_buffer) == 0) {
        strcpy(environment->executing_os_version, uname_buffer.sysname);
        strcat(environment->executing_os_version, " ");
        strcat(environment->executing_os_version, uname_buffer.release);
    } else {
        strcpy(environment->executing_os_version, "Apple Undefined");
    }
#endif

    return;
}

static void get_compiler_version(build_environment_t *environment)
{
#if defined(__clang__)
    char compiler_name[MAXIMUM_LENGTH_OF_COMPILER_NAME];
    strcpy(compiler_name, "Clang");
    sprintf(environment->compiler_version, "%s %s",
            compiler_name, __VERSION__);
#elif defined(__ICC) || defined(__INTEL_COMPILER)
    strcpy(compiler_name, "ICC");
    sprintf(environment->compiler_version, "%s %s",
            compiler_name, __VERSION__);
#elif defined(__GNUC__) || defined(__GNUG__)
    char compiler_name[MAXIMUM_LENGTH_OF_COMPILER_NAME];
#if defined(__GNUG__)
    strcpy(compiler_name, "g++");
#else
    strcpy(compiler_name, "gcc");
#endif
    sprintf(environment->compiler_version, "%s %s",
            compiler_name, __VERSION__);
#elif defined(__HP_cc) || defined(__HP_aCC)

#if defined(__HP_cc)
    int version = (int)(__HP_cc);
    sprintf(environment->compiler_version, "HP CC %d", version);
#else
    int version = (int)(__HP_aCC);
    sprintf(environment->compiler_version, "HP aCC %d", version);
#endif

#elif defined(__IBMC__) || defined(__IBMCPP__)
    sprintf(environment->compiler_version, "IBM XL %s", __xlC__);

#elif defined(_MSC_VER)
    int version = (int)(_MSC_VER);
    const int vc_plusplus_start_version = 800;
    const int vc_plusplus_step_version = 1000;

    if (version < vc_plusplus_start_version) {

        sprintf(environment->compiler_version, "C Compiler %d.%d",
                version / 100, version %100);

    } else if (version < vc_plusplus_step_version) {

        version -= (vc_plusplus_start_version - 100);

        sprintf(environment->compiler_version, "Visual C++ %d.%d",
                version / 100, version %100);

    } else {
        version -= (vc_plusplus_step_version - 400);

        sprintf(environment->compiler_version, "Visual C++ %d.%d",
                version / 100, version %100);

    }

#elif defined(__PGI)
    int major_version = (int)(__PGIC__);
    int minor_version = (int)(__PGIC_MINOR__);
    sprintf(environment->compiler_version, "Portland %d.%d",
            major_version, minor_version);

#elif defined(__SUNPRO_C) || defined(__SUNPRO_CC)
	/* Oracle Solaris Studio. ----------------------------------- */

#if defined(__SUNPRO_C)
    unsigned int version = (unsigned int)(__SUNPRO_C);
    sprintf(environment->compiler_version,
            "Oracle Solaris %x", version);
#else
    unsigned int version = (unsigned int)(__SUNPRO_CC);
    sprintf(environment->compiler_version,
            "Oracle Solaris %x", version);
#endif

#else
    sprintf(environment->compiler_version,
            "Undefined Compiler");
#endif

    return;
}

void get_build_environment(build_environment_t *environment)
{
    clear_build_environment_parameter(environment);

#ifdef WINDOWS_OS
    environment->os = OS_TYPE_WINDOWS;
#elif defined(APPLE_OS)
    #ifdef TARGET_OS_IPHONE
    environment->os = OS_TYPE_IOS;
    #elif TARGET_IPHONE_SIMULATOR
    environment->os = OS_TYPE_IOS_SIMULATOR;
    #elif TARGET_OS_MAC
    environment->os = OS_TYPE_MAC_OS;
    #else
    environment->os = OS_TYPE_UNDEFINED_APPLE;
    #endif
#elif defined(LINUX_OS)
    environment->os = OS_TYPE_LINUX;
#else
    environment->os = OS_TYPE_UNDEFINED;
#endif

#ifdef BUILDING_X86_64_SYSTEM
    environment->binary_building_bit = 64;
#elif defined(BUILDING_X86_32_SYSTEM)
    environment->binary_building_bit = 32;
#else
#endif
    get_compiler_version(environment);

    if (environment->os == OS_TYPE_WINDOWS) {
        get_windows_os_name(environment);
    } else if (environment->os == OS_TYPE_LINUX) {
        get_linux_os_name(environment);
    } else if (environment->os == OS_TYPE_MAC) {
        get_linux_os_name(environment);
    }

    return;
}

void output_build_environment_to_stream(std::ostream &stream, const build_environment_t *environment)
{

    switch (environment->os) {
        case OS_TYPE_WINDOWS:
            stream << "Windows,";
            break;

        case OS_TYPE_IOS:
            break;

        case OS_TYPE_IOS_SIMULATOR:
            break;

        case OS_TYPE_MAC:
            break;

        case OS_TYPE_UNDEFINED_APPLE:
            break;

        case OS_TYPE_LINUX:
            stream << "Linux,";
            break;

        case OS_TYPE_UNDEFINED:
            break;

    }
    stream << environment->binary_building_bit << " bit,";
    stream << environment->build_os_version << ",";
    stream << environment->compiler_version << ",";
    stream << environment->executing_os_version << ",";
    stream << environment->executing_architecture << ",";
    return;
}
