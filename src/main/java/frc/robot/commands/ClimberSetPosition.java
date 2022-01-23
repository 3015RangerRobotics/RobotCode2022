// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.State;

public class ClimberSetPosition extends CommandBase {

  boolean extend;

  /** Creates a new ClimberSetPosition. */
  public ClimberSetPosition(boolean extend) {

    this.extend = extend;

    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(extend) {
      RobotContainer.climber.setState(Climber.State.kExtend);
    } else {
      RobotContainer.climber.setState(Climber.State.kRetract);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setState(Climber.State.kDefault);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.climber.getState() == Climber.State.kDefault;
  }
}
