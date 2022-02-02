// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class PurgeBall extends CommandBase {
  public Intake intake;
  public Feeder feeder;

  // TODO: finish this
  /** Creates a new BallPurge. */
  public PurgeBall(int side) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.intake[side], RobotContainer.feeder[side],
    // RobotContainer.shooter[side]);
    intake = RobotContainer.intake[side];
    feeder = RobotContainer.feeder[side];
    addRequirements(intake, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.purge();
    feeder.setPercentOutput(Constants.FEEDER_PURGE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
