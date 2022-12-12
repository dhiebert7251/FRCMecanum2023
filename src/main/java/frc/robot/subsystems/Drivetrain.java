// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.ChassisConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

//SparkMAX libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;

//Sensor libraries
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

//Dashboard libraries
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Math/kinematics librarys
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;


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
  
    /** Creates a new Drivetrain. */
    public Drivetrain() {

      //Motors
      leftFrontMotor = new CANSparkMax(Constants.DriveConstants.LEFT_FRONT_MOTOR, MotorType.kBrushless);
      leftFrontMotor.restoreFactoryDefaults();
      leftFrontMotor.setInverted(Constants.DriveConstants.LEFT_FRONT_INVERTED);
      //leftFrontMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
      leftRearMotor = new CANSparkMax(Constants.DriveConstants.LEFT_REAR_MOTOR, MotorType.kBrushless);
      leftRearMotor.restoreFactoryDefaults();
      leftRearMotor.setInverted(Constants.DriveConstants.LEFT_REAR_INVERTED);
      //leftRearMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
      rightFrontMotor = new CANSparkMax(Constants.DriveConstants.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
      rightFrontMotor.restoreFactoryDefaults();
      rightFrontMotor.setInverted(Constants.DriveConstants.RIGHT_FRONT_INVERTED);
      //rightFrontMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
      rightRearMotor = new CANSparkMax(Constants.DriveConstants.RIGHT_REAR_MOTOR, MotorType.kBrushless);
      rightRearMotor.restoreFactoryDefaults();
      rightRearMotor.setInverted(Constants.DriveConstants.RIGHT_REAR_INVERTED);
      //rightRearMotor.getEncoder(Type.kHallSensor,DriveConstants.kEncoderCPR);
    
      //mecanum drivetrain
      drive = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
      setMaxSpeed();
      
      //Sensors

      //Gyro
      ADXRS450_Gyro gyro = new ADXRS450_Gyro();
      gyro.reset();

      //Odometry
      
      driveOdometry = new MecanumDriveOdometry(Constants.DriveConstants.KINEMATICS, getGyroRotation(), getWheelPositions());

      //Drive motor encoders
      leftFrontMotorEncoder = leftFrontMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,Constants.DriveConstants.kEncoderCPR);
      leftRearMotorEncoder = leftRearMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,Constants.DriveConstants.kEncoderCPR);
      rightFrontMotorEncoder = rightFrontMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,Constants.DriveConstants.kEncoderCPR);
      rightRearMotorEncoder = rightRearMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,Constants.DriveConstants.kEncoderCPR);

      /*
      TODO: why 10.71?  where is this conversion factor from?
       */
      leftFrontMotorEncoder.setPositionConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/10.71);
      leftRearMotorEncoder.setPositionConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/10.71);
      rightFrontMotorEncoder.setPositionConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/10.71);
      rightRearMotorEncoder.setPositionConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/10.71);
      leftFrontMotorEncoder.setVelocityConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/(10.71*60));
      leftRearMotorEncoder.setVelocityConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/(10.71*60));
      rightFrontMotorEncoder.setVelocityConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/(10.71*60));
      rightRearMotorEncoder.setVelocityConversionFactor(Constants.ChassisConstants.WHEEL_CIRCUM/(10.71*60));
      
      //Shuffleboard data  
      configureDriveShuffleboard();
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
      // Update odometry

      driveOdometry.update(getGyroRotation(), getWheelPositions()); 

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
    
    
    /**
    * Drive Mecanum style with/without FOD (Field Oriented Driving)
    * 
    * @param throttle speed along y axis (forward is positive) [-1.0, 1.0]
    * @param slide speed along x-axis (right is positive) [-1.0, 1.0]
    * @param rotation rotational speed (clockwise is positive) [-1.0, 1.0]
    * @param useFOD use Field Oriented Driving
    */ 
    public void driveWithJoysticks(double throttle, double slide, double rotation, boolean useFOD, int joystickType) { 
      if (useFOD) {
        if (joystickType==1){
          drive.driveCartesian(throttle, slide, rotation, getGyroRotation());
        }
        else
        {
          //update rotation for joystick mod
        } 
      }
      else 
      {
        if (joystickType==2){
        drive.driveCartesian(throttle, slide, rotation);
        }
        else
        {
          //update rotation for joystick mod
        }
      }
    }
  


    /**
    * Sets the drive motors to brake mode
    * @see #setCoast
    */
    public void setBrake() {
      leftFrontMotor.setIdleMode(IdleMode.kBrake);
      leftRearMotor.setIdleMode(IdleMode.kBrake);
      rightFrontMotor.setIdleMode(IdleMode.kBrake);
      rightRearMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Sets the drive motors to brake mode
    * @see #setBrake
    */
    public void setCoast() {
      leftFrontMotor.setIdleMode(IdleMode.kCoast);
      leftRearMotor.setIdleMode(IdleMode.kCoast);
      rightFrontMotor.setIdleMode(IdleMode.kCoast);
      rightRearMotor.setIdleMode(IdleMode.kCoast);
    }

    /** Set the max speed of the drivetrain between [0,1] */
    public void setMaxSpeed() {
     drive.setMaxOutput(Constants.DriveConstants.MAX_SPEED);
    }

    
    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      leftFrontMotorEncoder.setPosition(0);
      leftRearMotorEncoder.setPosition(0);
      rightFrontMotorEncoder.setPosition(0);
      rightRearMotorEncoder.setPosition(0);
    }


    /** Get the distance driven by the left side */
    public double getDistanceLeft() {
     return (leftFrontMotorEncoder.getPosition() + leftRearMotorEncoder.getPosition())/2.0;
    }

    /** Get the distance driven by the right sice */
    public double getDistanceRight() {
     return (rightFrontMotorEncoder.getPosition() + rightRearMotorEncoder.getPosition())/2.0;
    }
  
    /** Get the velocity of the left side */
    public double getVelocityLeft(){
     return (leftFrontMotorEncoder.getVelocity() + leftRearMotorEncoder.getVelocity())/2.0;
    }

    /** Get the velocity of the right side */
    public double getVelocityRight(){
     return (rightFrontMotorEncoder.getVelocity() + rightRearMotorEncoder.getVelocity())/2.0;
    }   

    /**
    * Returns the current wheel speeds of the drivetrain.
    */
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
     return new MecanumDriveWheelSpeeds(
          leftFrontMotorEncoder.getVelocity(),
          rightFrontMotorEncoder.getVelocity(),
          leftRearMotorEncoder.getVelocity(),
          rightRearMotorEncoder.getVelocity());
    }    
    
    public MecanumDriveWheelPositions getWheelPositions() {
      return new MecanumDriveWheelPositions(
          leftFrontMotorEncoder.getPosition(),
          rightFrontMotorEncoder.getPosition(),
          leftRearMotorEncoder.getPosition(),
          rightRearMotorEncoder.getPosition());
      }
    /** Get the current robot pose as a Pose2d object in meters */
    public Pose2d getPose() {
      return driveOdometry.getPoseMeters();
    }
  
    /** Get the current gyroscope angle in degrees from forward [-180, 180] */
    public double getGyroAngle() {
         return gyro.getRotation2d().getDegrees();
    }
    
    /** Get the current gyroscope velocity in degrees/second,
    * where positive is counter-clockwise */
    public double getGyroVelocity() {
     return -gyro.getRate();
    }

    /** Get the current gyroscope rotation as a Rotation2d object */
    public Rotation2d getGyroRotation() {
      return gyro.getRotation2d();
    }
    

      /** Reset the gyroscope to 0 */
    public void resetGyro() {
     if (getVelocityLeft() != 0 || getVelocityRight() != 0) {
      System.out.println("WARNING: Do not try to reset the gyroscope while the robot is moving");
      return;
    }
      gyro.reset();
    }

    public void calibrateGyro(){
      if (getVelocityLeft() != 0 || getVelocityRight() != 0) {
        System.out.println("WARNING: Do not try to reset the gyroscope while the robot is moving");
        return;
      }
      gyro.calibrate();
    }

    /**
     * TODO:  double check - i think this is redundant
    * Gets the current wheel speeds.
    *
    * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
      double leftFrontSpeed = (leftFrontMotorEncoder.getVelocity())*(Constants.ChassisConstants.WHEEL_CIRCUM)/60;
      double leftRearSpeed = (leftRearMotorEncoder.getVelocity())*(Constants.ChassisConstants.WHEEL_CIRCUM)/60;
      double rightFrontSpeed = (rightFrontMotorEncoder.getVelocity())*(Constants.ChassisConstants.WHEEL_CIRCUM)/60;
      double rightRearSpeed = (rightRearMotorEncoder.getVelocity())*(Constants.ChassisConstants.WHEEL_CIRCUM)/60;

      return new MecanumDriveWheelSpeeds(leftFrontSpeed, leftRearSpeed, rightFrontSpeed, rightRearSpeed);
    }

*/

    public void stop() {
      drive.stopMotor();

    }

    public void update() {
      //potentially used to update shuffleboard sensor data

    }
    
    /*
     * TODO: need to see how this all works
     */
    private void configureDriveShuffleboard() {
      //set up shuffleboard tab to monitor/control drive components
      ShuffleboardTab shuffleDriveTab = Shuffleboard.getTab("Drive");

      //Add motor encoder monitors
      shuffleDriveTab
        .add("Left Front Motor", leftFrontMotor)
        .withPosition(0,0)
        .withSize(2,1);

      shuffleDriveTab
        .add("Right Front Motor", rightFrontMotor)
        .withPosition(3,0)
        .withSize(2,1);

      shuffleDriveTab
        .add("Left Rear Motor", leftRearMotor)
        .withPosition(0,2)
        .withSize(2,1);

      shuffleDriveTab
        .add("Right Rear Motor", rightRearMotor)
        .withPosition(3,2)
        .withSize(2,1);

      SmartDashboard.putString("Gyro Angle", Double.toString(gyro.getAngle()));


      SmartDashboard.putNumber("Front Left Drive", leftFrontMotor.getOutputCurrent());
      SmartDashboard.putNumber("Back Left Drive", leftRearMotor.getOutputCurrent());
      SmartDashboard.putNumber("Front Right Drive", rightFrontMotor.getOutputCurrent());
      SmartDashboard.putNumber("Back Right Drive", rightRearMotor.getOutputCurrent());

      
    }
}
