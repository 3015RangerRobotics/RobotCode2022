// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase {
  private Intake intake;
  private Feeder feeder;

  /** Creates a new IntakeBall. Will run eternally */
  public IntakeBall(int side) {
    // there has to be a better way to do this
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
    //TODO:set intake to stop when BOTH feeder sensor is true and intake sensor is true
    intake.intake();

    //TODO:set feeder motor to stop when feeder sensor is tripped
    feeder.setPercentOutput(Constants.FEEDER_INTAKE_SPEED);
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
    return false;//TODO:end command when both feeder sensor is true and intake sensor is true
  }
}
