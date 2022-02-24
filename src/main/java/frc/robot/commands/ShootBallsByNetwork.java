// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Sets the RPM from the network table */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootBallsByNetwork extends CommandBase {
  private Intake intake;
  private Feeder feeder;
  private Shooter shooter;
  double rpm;
  double loopDelay;
  double initialDelay;
  Timer timer;

  public ShootBallsByNetwork(int side) {
    this(side, 0.0, 0.0);
  }

  /** Creates a new ShootBalls. */
  public ShootBallsByNetwork(int side, double loopDelay, double initialDelay) {
    this.intake = RobotContainer.intake[side];
    this.feeder = RobotContainer.feeder[side];
    this.shooter = RobotContainer.shooter[side];
    SmartDashboard.putNumber("shooter speed", 0);
    this.rpm = SmartDashboard.getNumber("shooter speed", 0);
    this.timer = new Timer();
    this.loopDelay = loopDelay < 0.25 ? 0.25 : loopDelay;
    addRequirements(intake, feeder, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    this.rpm = SmartDashboard.getNumber("shooter speed", 0);
    shooter.setRPM(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRPM(rpm);
    double time = timer.get() - initialDelay;
    if (time < 0) {
      return;
    }
    double cycle = timer.get() % loopDelay;
    if (cycle < 0.25) {
      feeder.setPercentOutput(Constants.FEEDER_SHOOT_SPEED);
      intake.intake();
    } else {
      feeder.setPercentOutput(0);
      intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    shooter.stop();
    feeder.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
