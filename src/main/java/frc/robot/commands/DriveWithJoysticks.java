// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain m_driveTrain;
  private final XboxController m_driverJoystick;

  
  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drivetrain driveTrain, XboxController driverJoystick) {
    m_driveTrain = driveTrain;
    m_driverJoystick = driverJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double ySpeed = m_driverJoystick.getLeftY()*Constants.DRIVETRAIN_SPEED;
    double xSpeed = m_driverJoystick.getLeftX()*Constants.DRIVETRAIN_SPEED;
    double zRotation = m_driverJoystick.getRightX();  //add rotation scale constant?

    m_driveTrain.driveWithJoysticks(ySpeed, xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_driveTrain.driveWithJoysticks(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
