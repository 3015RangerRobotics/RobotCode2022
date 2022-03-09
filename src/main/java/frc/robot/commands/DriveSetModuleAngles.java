// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveSetModuleAngles extends CommandBase {

  double angle;
  double timeout;
  Timer timer;

  public DriveSetModuleAngles() {
    this(0, 0.15);
  }

  public DriveSetModuleAngles(double angle) {
    this(angle, 0.15);
  }

  /** Creates a new DriveSetModuleAngles. */
  public DriveSetModuleAngles(double angle, double timeout) {
    addRequirements(RobotContainer.drive);
    this.angle = angle;
    this.timeout = timeout;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    RobotContainer.drive.setModuleRotation(angle);
  }

  public boolean isFinished() {
    return timer.hasElapsed(timeout);
  }
}
