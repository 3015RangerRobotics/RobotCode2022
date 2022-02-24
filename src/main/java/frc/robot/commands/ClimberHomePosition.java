// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Climber Homeing for Climbing Sequential Command */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimberHomePosition extends CommandBase {

  double threshold = 10; // set this to some reasonable number

  /**
   * Creates a new ClimberHomePosition. Finishes when the climber is homed or when
   * the climber motor experiences an unexpected load
   */
  public ClimberHomePosition() {
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.climber.hasBeenHomed()) {
      return;
    }
    RobotContainer.climber.setOutput(-.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted && (RobotContainer.climber.getBottomLimit() || RobotContainer.climber.getClimberCurrent() > threshold)) {
      RobotContainer.climber.setSensorZero();
      RobotContainer.climber.setHasBeenHomed();
    }
    RobotContainer.climber.setOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.climber.getBottomLimit() || RobotContainer.climber.getClimberCurrent() > threshold
       || RobotContainer.climber.hasBeenHomed();

  }
}
