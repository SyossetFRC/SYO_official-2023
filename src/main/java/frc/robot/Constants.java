// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.51;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.51;

    // public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 9; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 14; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 50; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(284.1); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 16; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 53; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(261.6); 
       
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 51; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(303.3); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 18; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 52; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(5.9); 

    public static final int ELEVATOR_PULLEY_MOTOR_1 = 22;
    public static final int ELEVATOR_PULLEY_MOTOR_2 = 23;
    public static final int ELEVATOR_SWITCH_TOP = 1;
    public static final int ELEVATOR_SWITCH_BOTTOM = 2;

    public static final int WINCH_MOTOR_1 = 61;
    public static final int WINCH_MOTOR_2 = 7;
    public static final int WINCH_ENCODER = 0;
    public static final int WINCH_SWITCH_TOP = 3;
    public static final int WINCH_SWITCH_BOTTOM = 4;

    public static final int INTAKE_MOTOR_1 = 40;
    public static final int INTAKE_MOTOR_2 = 41;
    public static final int CLAW_SOLENOID_FORWARD = 0;
    public static final int CLAW_SOLENOID_REVERSE = 1;
    public static final int RELEASE_SOLENOID_1 = 2;
    public static final int RELEASE_SOLENOID_2 = 3;

    public static final int BLINKIN = 0;
}