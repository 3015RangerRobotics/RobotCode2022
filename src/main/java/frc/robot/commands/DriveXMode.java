// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveXMode extends CommandBase {

  static int[] angles = {45, 135, 225, 315};

  /** Creates a new DriveXMode. */
  public DriveXMode() {
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.enableBrakeMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.setModuleDrivePct(0);
    for (int i = 0; i < 4; i++) {
      RobotContainer.drive.setModuleRotation(angles[i], i);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.setBrakeModes(new boolean[] {false, true, false, true});
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
