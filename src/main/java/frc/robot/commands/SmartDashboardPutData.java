// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SmartDashboardPutData extends InstantCommand {
  String key;
  Object value;


  /** Creates a new SmartDashboardPutData. */
  public SmartDashboardPutData(String key, Object value) {
    this.key = key;
    this.value = value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (value instanceof Number) {
      SmartDashboard.putNumber(key, (double) value);
      return;
    }
    if (value instanceof Boolean) {
      SmartDashboard.putBoolean(key, (boolean) value);
    }
    if (value instanceof String) {
      SmartDashboard.putString(key, (String) value);
    }
    if (value instanceof Sendable) {
      SmartDashboard.putData(key, (Sendable) value);
    }
  }

  public boolean runsWhenDisables() {
    return true;
  }
}
