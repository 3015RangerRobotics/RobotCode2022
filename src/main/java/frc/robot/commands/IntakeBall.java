// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeBall extends CommandBase {
  private int side;

  /** Creates a new IntakeBall. Will run eternally */
  public IntakeBall(int side) {
    addRequirements(RobotContainer.intake[side], RobotContainer.feeder[side]);
    this.side = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake[side].intake();
    if (RobotContainer.feeder[side].getBallDetector()) {
      RobotContainer.feeder[side].setPercentOutput(0);
    } else {
      RobotContainer.feeder[side].setPercentOutput(Constants.FEEDER_INTAKE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake[side].stop();
    RobotContainer.feeder[side].setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
