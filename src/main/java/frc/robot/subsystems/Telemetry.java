// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telemetry extends SubsystemBase {
  /** Creates a new Telemetry. */
  public Telemetry() {

;
  }

  @Override
  public void periodic() {
    try (PowerDistribution pdp = new PowerDistribution()) {
      // This method will be called once per scheduler run
          // Get the current going through channel 7, in Amperes.
      // The PDP returns the current in increments of 0.125A.
      // At low currents the current readings tend to be less accurate.
      double current7 = pdp.getCurrent(7);
      SmartDashboard.putNumber("Current Channel 7", current7);

      // Get the voltage going into the PDP, in Volts.
      // The PDP returns the voltage in increments of 0.05 Volts.
      double voltage = pdp.getVoltage();
      SmartDashboard.putNumber("Voltage", voltage);

      // Retrieves the temperature of the PDP, in degrees Celsius.
      double temperatureCelsius = pdp.getTemperature();
      SmartDashboard.putNumber("Temperature", temperatureCelsius);

      // Get the total current of all channels.
      double totalCurrent = pdp.getTotalCurrent();
      SmartDashboard.putNumber("Total Current", totalCurrent);

      // Get the total power of all channels.
      // Power is the bus voltage multiplied by the current with the units Watts.
      double totalPower = pdp.getTotalPower();
      SmartDashboard.putNumber("Total Power", totalPower);

      // Get the total energy of all channels.
      // Energy is the power summed over time with units Joules.
      double totalEnergy = pdp.getTotalEnergy();
      SmartDashboard.putNumber("Total Energy", totalEnergy);
    }

  }
}
