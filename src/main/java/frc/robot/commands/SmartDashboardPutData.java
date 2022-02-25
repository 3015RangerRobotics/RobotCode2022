// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SmartDashboardPutData extends InstantCommand {
  String type;
  String key;
  Object value;


  /** Creates a new SmartDashboardPutData. */
  public SmartDashboardPutData(String type, String key, Object value) {
    this.type = type;
    this.key = key;
    this.value = value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(type) {
      case "String":
        SmartDashboard.putString(key, (String) value);
        break;
      case "double":
        SmartDashboard.putNumber(key, (double) value);
        break;
      case "int":
        SmartDashboard.putNumber(key, (int) value);
        break;
      case "boolean":
        SmartDashboard.putBoolean(key, (boolean) value);
        break;
      default:
        SmartDashboard.putData(key, (Sendable) value);
        break;
    }
  }

  public boolean runsWhenDisables() {
    return true;
  }
}
