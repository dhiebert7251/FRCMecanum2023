// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain driveTrain;
  private DoubleSupplier throttle;
  private DoubleSupplier slide;
  private DoubleSupplier rotation;
  private boolean useFOD;
  private int joystickType;


  
  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drivetrain driveTrain, DoubleSupplier throttle, DoubleSupplier slide, DoubleSupplier rotation, boolean useFOD, int joystickType) {
    this.driveTrain = driveTrain;
    this.throttle = throttle;
    this.slide = slide;
    this.rotation = rotation;
    this.useFOD = useFOD;
    this.joystickType = 1;
    addRequirements(this.driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Drive the robot using driveCartesian style control
    // NOTE: The +/- for the y axis on the joystick is inverted from
    // the right-hand-rule
    // TODO: update joystickType to get value from dashboard
    driveTrain.driveWithJoysticks(
        -throttle.getAsDouble(),
        slide.getAsDouble(),
        rotation.getAsDouble(),
        useFOD,
        1);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set joystick inputs to 0 for safety
    driveTrain.driveWithJoysticks(0.0, 0.0, 0.0, true,1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
