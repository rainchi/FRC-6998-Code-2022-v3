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
    public static final boolean DEBUG = true;

    // Motor channel constants
    // Chassis (FL: Front Left FR: Front Right RL: Rear Left RR: Rear Right)
    public static final int MOTOR_CHASSIS_FL = 1;
    public static final int MOTOR_CHASSIS_FR = 2;
    public static final int MOTOR_CHASSIS_RL = 3;
    public static final int MOTOR_CHASSIS_RR = 4;
    // Hang
    public static final int MOTOR_HANG_LEFT = 5;
    public static final int MOTOR_HANG_CENTER = 6;
    public static final int MOTOR_HANG_RIGHT = 7;
    // Shoot
    public static final int MOTOR_SHOOT_MAIN = 1;
    public static final int MOTOR_SHOOT_AUXILIARY = 8;
    public static final int MOTOR_SHOOT_ROTATE = 9;
    public static final int MOTOR_SHOOT_ANGLE = 10;
    public static final int MOTOR_SHOOT_TRANSFER = 3;
    // Collect
    public static final int MOTOR_INTAKE = 2;

    // Motor Invert constants
    // Chassis
    public static final boolean MOTOR_CHASSIS_FL_INVERTED = false;
    public static final boolean MOTOR_CHASSIS_FR_INVERTED = true;
    public static final boolean MOTOR_CHASSIS_RL_INVERTED = false;
    public static final boolean MOTOR_CHASSIS_RR_INVERTED = true;
    // Hang
    public static final boolean MOTOR_HANG_LEFT_INVERTED = false;
    public static final boolean MOTOR_HANG_CENTER_INVERTED = false;
    public static final boolean MOTOR_HANG_RIGHT_INVERTED = true;
    // Shoot
    public static final boolean MOTOR_SHOOT_MAIN_INVERTED = false;
    public static final boolean MOTOR_SHOOT_AUXILIARY_INVERTED = false;
    public static final boolean MOTOR_SHOOT_ROTATE_INVERTED = false;
    public static final boolean MOTOR_SHOOT_ANGLE_INVERTED = true;
    public static final boolean MOTOR_SHOOT_TRANSFER_INVERTED = true;
    // Intake
    public static final boolean MOTOR_INTAKE_INVERTED = true;

    // Limit switch channel constants
    public static final int LIMIT_HANG_LEFT = 0;
    public static final int LIMIT_HANG_CENTER = 1;
    public static final int LIMIT_HANG_RIGHT = 2;

    public static final float SOFT_LIMIT_LEFT_MAX = 137;
    public static final float SOFT_LIMIT_RIGHT_MAX = 137;
    public static final float SOFT_LIMIT_CENTER_MAX = 250;

    // Solenoid channel constants
    public static final int SOLENOID_HANG_FORWARD = 2;
    public static final int SOLENOID_HANG_REVERSE = 3;
    public static final int SOLENOID_INTAKE_FORWARD = 0;
    public static final int SOLENOID_INTAKE_REVERSE = 1;

    // Chassis constants
    public static final double CHASSIS_DEADLINE = 0.15;
    public static final double CHASSIS_GEARING = 9.52;
    public static final double DISTANCE_METER_PER_ROTATION = 4 * 2.54 / 100.0 * Math.PI; // Wheel Diameter(Inches) * 2.54(inch to cm) / 100.0(cm to meter) * math.PI(Diameter to Wheel Perimeter)

    // FeedForward constants for motors
    public static final double F_SHOOT_MAIN = 0.073;
    public static final double F_SHOOT_AUXILIARY = 0.0002;
    public static final double FF_CHASSIS_ksVolts = 0.1047;
    public static final double FF_CHASSIS_kvVoltSecondsPerMeter = 3.6991;
    public static final double FF_CHASSIS_kaVoltSecondsSquaredPerMeter = 0.49561;

    // PID constants for motors
    public static final double[] PID_CHASSIS = {1.9919, 0, 0};
    public static final double[] PID_HANG_LEFT = {0.5, 0, 0};
    public static final double[] PID_HANG_CENTER = {0.5, 0, 0};
    public static final double[] PID_HANG_RIGHT = {0.5, 0, 0};
    public static final double[] PID_SHOOT_MAIN = {0.7, 0, 0};
    public static final double[] PID_SHOOT_AUXILIARY = {0.0006, 0, 0};
    public static final double[] PID_SHOOT_ROTATE = {0.02, 0, 0};
    public static final double[] PID_SHOOT_ROTATE_FAST = {0.5,0,0}; // Calculate output according to navx turn rate
    public static final double[] PID_SHOOT_ANGLE = {0.05, 0, 0};

    // Max positions for motors
    public static final double POSITION_LEFT_HANG_MAX = 0;
    public static final double POSITION_CENTER_HANG_MAX = 0;
    public static final double POSITION_RIGHT_HANG_MAX = 0;

    // Auto alignment
    public static final double AUTO_ALIGNMENT_X_OFFSET = 0;
    public static final double AUTO_ALIGNMENT_Y_OFFSET = 0;
    public static final double AUTO_ALIGNMENT_MOUNT_ANGLE = 27;
    public static final double AUTO_ALIGNMENT_LENS_HEIGHT_METER = 1.07;
    public static final double AUTO_ALIGNMENT_GOAL_HEIGHT_METER = 2.5;


    // Wheel rpm(not motor rpm)
    public static final int SHOOT_RPM = 6000;
    // Shooter gearing(wheel rpm/motor rpm)
    public static final double SHOOT_GEARING = 3;
}
