// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Uses the Controller D pad to move the shooters' hood */

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodDPad extends CommandBase {
  double hoodPos;

  /** Creates a new HoodDPad. */
  public HoodDPad() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int hat = RobotContainer.getDriverDPad();
    double speed = 0;
    switch (hat) {
      case 0:
        speed = 0.2;
        break;
      case 180:
        speed = -0.2;
        break;
      default:
        break;
    }
    RobotContainer.hood.setHoodOutputPercentage(speed);
    SmartDashboard.putNumber("Hood Position", RobotContainer.hood.getHoodPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.hood.setHoodOutputPercentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
