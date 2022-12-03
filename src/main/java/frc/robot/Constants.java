// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Motor constants 
  /*
  TODO:
   Get CANbus ID values for sparkmax controllers
  */
  public static final int LEFT_FRONT_MOTOR = 0;
  public static final int LEFT_REAR_MOTOR = 1;
  public static final int RIGHT_FRONT_MOTOR = 2;
  public static final int RIGHT_REAR_MOTOR = 3;

  //Controller constants

  //controller - Driver
  public static final int DRIVER_JOYSTICK = 0;

  //controller - Driver Axis maps
  public static final int DRIVER_JOYSTICK_LEFT_X_AXIS = 0;
  public static final int DRIVER_JOYSTICK_LEFT_Y_AXIS = 1;
  /*
  Modified controller - right joystick replaced with potentiometer

  public static final int DRIVER_JOYSTICK_RIGHT_X_AXIS = 2;
  public static final int DRIVER_JOYSTICK_RIGHT_Y_AXIS = 3;

  */
  public static final int DRIVER_ROTATION = 2; //replaces right x-axis


  //controller - Driver Button maps
  public static final int DRIVER_LEFT = 1;
  public static final int DRIVER_RIGHT = 3;
  public static final int DRIVER_UP = 4;
  public static final int DRIVER_DOWN = 2;
  public static final int DRIVER_SHOULDER_TOP_LEFT = 5;
  public static final int DRIVER_SHOULDER_TOP_RIGHT = 6;
  public static final int DRIVER_SHOULDER_BOTTOM_LEFT = 7;
  public static final int DRIVER_SHOULDER_BOTTOM_RIGHT = 8;
  public static final int DRIVER_LEFT_JOYSTICK = 9;
  //public static final int DRIVER_RIGHT_JOYSTICK = 10; //removed with joystick modification

  //drivetrain speed constants
  public static final double DRIVETRAIN_SPEED = 1;

  //controller - operator
  public static final int OPERATOR_JOYSTICK = 1;

  //controller - Operator Axis maps
  public static final int OPERATOR_JOYSTICK_LEFT_X_AXIS = 0;
  public static final int OPERATOR_JOYSTICK_LEFT_Y_AXIS = 1;
  public static final int OPERATOR_JOYSTICK_RIGHT_X_AXIS = 2;
  public static final int OPERATOR_JOYSTICK_RIGHT_Y_AXIS = 3;

  //controller - Driver Button maps
  public static final int OPERATOR_LEFT = 1;
  public static final int OPERATOR_RIGHT = 3;
  public static final int OPERATOR_UP = 4;
  public static final int OPERATOR_DOWN = 2;
  public static final int OPERATOR_SHOULDER_TOP_LEFT = 5;
  public static final int OPERATOR_SHOULDER_TOP_RIGHT = 6;
  public static final int OPERATOR_SHOULDER_BOTTOM_LEFT = 7;
  public static final int OPERATOR_SHOULDER_BOTTOM_RIGHT = 8;
  public static final int OPERATOR_LEFT_JOYSTICK = 9;
  public static final int OPERATOR_RIGHT_JOYSTICK = 10;

  public static final class DriveConstants {
    /*
    TODO:
    update values to actual robot
    */
    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot in m
    public static final double kWheelBase = 0.7;
    // Distance between centers of front and back wheels on robot in m

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 42; //updated to reflect NEO Hall-Effect encoders CPR
    public static final double kWheelDiameterMeters = 0.1524; //6" wheel = 0.1524 m
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        kWheelCircumference / (double) kEncoderCPR;


    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    /* 
    TODO:
    Example value only - as above, this must be tuned for your drive!
    Set values to be updated through smartdashboard for easier PID tuning
    */
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;
  }

}