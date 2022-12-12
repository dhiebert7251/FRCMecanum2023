// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//subsystems
import frc.robot.subsystems.Drivetrain;

//commands
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.Autonomous.autoBoxLeft;
import frc.robot.commands.Autonomous.autoBoxRight;
import frc.robot.commands.Autonomous.autoDoNothing;
import frc.robot.commands.Autonomous.autoDriveForward;
import frc.robot.commands.Autonomous.autoMecanumY;

//dashboard
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Joystick declare
    private final XboxController  driverJoystick = new XboxController(Constants.Controllers.DRIVER_JOYSTICK);
    //private final XboxController operatorJoystick = new XboxController(Constants.Controllers.OPERATOR_JOYSTICK);
 
  //Subsystems declare
    private final Drivetrain driveTrain = new Drivetrain();

  //Commands declare
    private final DriveWithJoysticks driveWithJoysticks 
      = new DriveWithJoysticks(
          driveTrain,
          () -> driverJoystick.getLeftY(),
          () -> driverJoystick.getLeftX(),
          () -> driverJoystick.getRightX(),
          false,
          1);

    private final DriveWithJoysticks driveWithJoysticksFOD 
      = new DriveWithJoysticks(
          driveTrain,
          () -> driverJoystick.getLeftY(),
          () -> driverJoystick.getLeftX(),
          () -> driverJoystick.getRightX(),
          true,
          1);

     
  //Sendable chooser declare
    SendableChooser<Command> autoChooser = new SendableChooser<>();  //allows for autonomous selection
    


  public RobotContainer() {

    //set autonomous selector
    autoChooser.setDefaultOption(
      "Do Nothing", 
      new autoDoNothing()
      );

    autoChooser.addOption(
      "Drive Forward",
      new autoDriveForward()
      );

    autoChooser.addOption(
      "Box Left",
      new autoBoxLeft()
      );

    autoChooser.addOption(
      "Box Right",
      new autoBoxRight()
      );

    autoChooser.addOption(
      "Mecanum Y",
      new autoMecanumY()
      );

      SmartDashboard.putNumber("Controller Angle", driverJoystick.getRightX());

    // set default commands on subsystems
    driveTrain.setDefaultCommand(driveWithJoysticks);


    // Configure the button bindings
    configureButtonBindings();

    /*
     * TODO: should this be here?
     */

    //Initialize/calibrate gyro
    driveTrain.resetGyro();
    driveTrain.calibrateGyro();
  }




  private void configureButtonBindings() {
    //Configure dashboard button for FOD


    /*
     * TODO: find values for dpad; create command (method?) to face 0, 90, 180, 270
     */

    


    //JoystickButton driverLeft = new JoystickButton(driverJoystick, Constants.DRIVER_LEFT);
    //JoystickButton driverRight = new JoystickButton(driverJoystick, Constants.DRIVER_RIGHT);
    //JoystickButton driverUp = new JoystickButton(driverJoystick, Constants.DRIVER_UP);
    //JoystickButton driverDown = new JoystickButton(driverJoystick, Constants.DRIVER_DOWN);
    //JoystickButton driverShoulderTopLeft = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_TOP_LEFT);
    //JoystickButton driverShoulderTopRight = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_TOP_RIGHT);
    //JoystickButton driverShoulderBottomLeft = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_BOTTOM_LEFT);
    //JoystickButton driverShoulderBottomRight = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_BOTTOM_RIGHT);
    //JoystickButton driverLeftJoystick = new JoystickButton(driverJoystick, Constants.DRIVER_LEFT_JOYSTICK);
    //JoystickButton driverRightJoystick = new JoystickButton(driverJoystick, Constants.DRIVER_RIGHT_JOYSTICK);

    //JoystickButton operatorLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_LEFT);
    //JoystickButton operatorRight = new JoystickButton(operatorJoystick, Constants.OPERATOR_RIGHT);
    //JoystickButton operatorUp = new JoystickButton(operatorJoystick, Constants.OPERATOR_UP);
    //JoystickButton operatorDown = new JoystickButton(operatorJoystick, Constants.OPERATOR_DOWN);
    //JoystickButton operatorShoulderTopLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_TOP_LEFT);
    //JoystickButton operatorShoulderTopRight = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_TOP_RIGHT);
    //JoystickButton operatorShoulderBottomLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_BOTTOM_LEFT);
    //JoystickButton operatorShoulderBottomRight = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_BOTTOM_RIGHT);
    //JoystickButton operatorMidLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_MID_LEFT);
    //JoystickButton operatorLeftJoystick = new JoystickButton(operatorJoystick, Constants.OPERATOR_LEFT_JOYSTICK);
    //JoystickButton operatorRightJoystick = new JoystickButton(operatorJoystick, Constants.OPERATOR_RIGHT_JOYSTICK);
  
      //button command links

      /*  example
      operatorUp.whenPressed(new ShootHigh(shooter)); //set shooter motor to shoot to high goal
      */
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return m_autoCommand;  //return the name of the command for autonomous
    return autoChooser.getSelected();

  }
}
