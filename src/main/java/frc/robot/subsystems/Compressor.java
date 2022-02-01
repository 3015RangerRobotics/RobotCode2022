// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compressor extends SubsystemBase {
  PneumaticHub pneumaticHub;
  edu.wpi.first.wpilibj.Compressor compressor;

  /** Creates a new Compressor. */
  public Compressor() {
    pneumaticHub = new PneumaticHub();
    compressor = pneumaticHub.makeCompressor();
    setCompressorEnabled(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Pressure (psi)", (int) getPressure());
  }

  public void setCompressorEnabled(boolean enabled) {
    if (enabled) {
      pneumaticHub.enableCompressorAnalog(95, 120);
    } else {
      pneumaticHub.disableCompressor();
    }
  }

  public double getPressure() {
    return pneumaticHub.getPressure(0); // Might be wrong channel, TBD
  }
}
