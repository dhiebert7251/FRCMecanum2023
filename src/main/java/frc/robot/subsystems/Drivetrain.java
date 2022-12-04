// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ChassisConstants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

//SparkMAX libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;



//Dashboard libraries
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Math/kinematics librarys
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;


public class Drivetrain extends SubsystemBase {


  //drivetrain speed controllers   
   
  CANSparkMax leftFrontMotor;
  CANSparkMax rightFrontMotor;
  CANSparkMax leftRearMotor;
  CANSparkMax rightRearMotor;

  //drivetrain type
  MecanumDrive drive;

  //odometry
  MecanumDriveOdometry driveOdometry;
  
  //sensors
  Gyro gyro;

  RelativeEncoder leftFrontMotorEncoder;
  RelativeEncoder rightFrontMotorEncoder;
  RelativeEncoder leftRearMotorEncoder;
  RelativeEncoder rightRearMotorEncoder;

  /* 
  SparkMaxRelativeEncoder leftFrontMotorEncoder;
  SparkMaxRelativeEncoder rightFrontMotorEncoder;
  SparkMaxRelativeEncoder leftRearMotorEncoder;
  SparkMaxRelativeEncoder rightRearMotorEncoder;
  */
  
    /** Creates a new Drivetrain. */
    public Drivetrain() {

      /* 
      TODO:
        set motor constants to CAN bus addresses
        may need to reverse inverted motors

      */

      //Motors
      leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR, MotorType.kBrushless);
      leftFrontMotor.setInverted(true);
      //leftFrontMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
      leftRearMotor = new CANSparkMax(Constants.LEFT_REAR_MOTOR, MotorType.kBrushless);
      leftRearMotor.setInverted(true);
      //leftRearMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
      rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
      rightFrontMotor.setInverted(false);
      //rightFrontMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
      rightRearMotor = new CANSparkMax(Constants.RIGHT_REAR_MOTOR, MotorType.kBrushless);
      rightRearMotor.setInverted(false);
      //rightRearMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
    
      //mecanum drivetrain
      drive = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
      
      //Sensors

      //Gyro
      ADXRS450_Gyro gyro = new ADXRS450_Gyro();

      //Drive motor encoders
      leftFrontMotorEncoder = (SparkMaxRelativeEncoder) leftFrontMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,DriveConstants.kEncoderCPR);
      leftRearMotorEncoder = (SparkMaxRelativeEncoder) leftRearMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,DriveConstants.kEncoderCPR);
      rightFrontMotorEncoder = (SparkMaxRelativeEncoder) rightFrontMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,DriveConstants.kEncoderCPR);
      rightRearMotorEncoder = (SparkMaxRelativeEncoder) rightRearMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,DriveConstants.kEncoderCPR);



      //set distance per pulse for encoders     

      //Odometry class for tracking robot pose
      //MecanumDriveOdometry odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d(), initialPoseMeters)
      //MecanumDriveOdometry odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d());






      //Shuffleboard data  

      /*
      TODO:
        update SmartDashboard data to Shuffleboard data

      */

      Shuffleboard.getTab("Telemetry").add(gyro);
      SmartDashboard.putNumber("Front Left Position",leftFrontMotorEncoder.getPosition());
      SmartDashboard.putNumber("Front Left Velocity",leftFrontMotorEncoder.getVelocity());
      SmartDashboard.putNumber("Front Right Position",rightFrontMotorEncoder.getPosition());
      SmartDashboard.putNumber("Front Right Velocity",rightFrontMotorEncoder.getVelocity());
      SmartDashboard.putNumber("Rear Left Position",leftRearMotorEncoder.getPosition());
      SmartDashboard.putNumber("Rear Left Velocity",leftRearMotorEncoder.getVelocity());       
      SmartDashboard.putNumber("Rear Right Position", rightRearMotorEncoder.getPosition());
      SmartDashboard.putNumber("Rear Right Velocity",rightRearMotorEncoder.getVelocity());     


    }


    @Override
    public void periodic() {
      // This method will be called once per scheduler run

    }


  /*
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
    
    
    public void driveWithJoysticks(double ySpeed, double xSpeed, double zRotation) {
      drive.driveCartesian(ySpeed, xSpeed, zRotation);
    }



    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      leftFrontMotorEncoder.setPosition(0);
      leftRearMotorEncoder.setPosition(0);
      rightFrontMotorEncoder.setPosition(0);
      rightRearMotorEncoder.setPosition(0);
    }

    /**
    * Gets the front left drive encoder.
    *
    * @return the front left drive encoder
    */

    public SparkMaxRelativeEncoder getLeftFrontEncoder(){
      return leftFrontMotorEncoder;
    }

    public SparkMaxRelativeEncoder getLeftRearEncoder(){
      return leftRearMotorEncoder;
    }

    public SparkMaxRelativeEncoder getRightFrontEncoder(){
      return rightFrontMotorEncoder;
    }

    public SparkMaxRelativeEncoder getRightRearEncoder(){
      return rightRearMotorEncoder;
    }

    /**
    * Gets the current wheel speeds.
    *
    * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
    */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
      double leftFrontSpeed = (leftFrontMotorEncoder.getVelocity())*(DriveConstants.kWheelCircumference)/60;
      double leftRearSpeed = (leftRearMotorEncoder.getVelocity())*(DriveConstants.kWheelCircumference)/60;
      double rightFrontSpeed = (rightFrontMotorEncoder.getVelocity())*(DriveConstants.kWheelCircumference)/60;
      double rightRearSpeed = (rightRearMotorEncoder.getVelocity())*(DriveConstants.kWheelCircumference)/60;

      return new MecanumDriveWheelSpeeds(leftFrontSpeed, leftRearSpeed, rightFrontSpeed, rightRearSpeed);
    }



    public void stop() {
      drive.stopMotor();

    }

    public void update() {
      //potentially used to update shuffleboard sensor data

    }
    
}
