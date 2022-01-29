// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compressor extends SubsystemBase {
  //TODO: use this class as a place for a PneumaticHub object,
  //TODO: create instant COMmANDS to turn compressor on and offf
  /** Creates a new Compressor. */
  public Compressor() {
    //TODO: call the enable compressor method(see below)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO: call the pneumatic pressure sensor though PH object and push to SmartDashboard(use math.round, don't need decimalgi)
  }
  //TODO: have accessor functions to turn said compressor on and off(.enableCompressorAnalog(),disableCompressor())
}
