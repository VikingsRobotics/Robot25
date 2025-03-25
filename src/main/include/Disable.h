#pragma once

/* Disable Subsystem Defines */

/* Disable swerve drive and pathplanner */
//#define NO_SWERVE 1
/* Disable the full elevator grabber and height*/
//#define NO_ELEVATOR_ARM 1
/* Disable part of elevator */
//#define NO_ELEVATOR 1
//#define NO_ARM 1
//#define NO_ROLLER 1
/* Compound define expansion */
#if defined(NO_ELEVATOR_ARM) || NO_ELEVATOR_ARM == 1
    #if !defined(NO_ELEVATOR)
        #define NO_ELEVATOR 1
    #endif
    #if !defined(NO_ARM)
        #define NO_ARM 1
    #endif
#endif

/* Disable Command Defines */

#if defined(NO_SWERVE)
    #define NO_SWERVE_JOYSTICK_COMMAND
    #define NO_SWERVE_CONTROLLER_COMMAND
    #define NO_SWERVE_AUTO_COMMAND
    #define NO_SWERVE_SYSID_COMMAND
    #define NO_SWERVE_ALIGN_APRILTAG_COMMAND
#endif

#if defined(NO_ELEVATOR)
    #define NO_ELEVATOR_HEIGHT_COMMAND
#endif

#if defined(NO_ARM)
    #define NO_ARM_ROTATION_COMMAND
#endif

#if defined(NO_ROLLER)

#endif

#define NO_SWERVE_ALIGN_APRILTAG_COMMAND
