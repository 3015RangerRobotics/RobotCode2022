// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Sequential Command Group For Climbing Sequence */

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAllTheWay extends SequentialCommandGroup {
  /** Creates a new ClimbAllTheWay. */
  public ClimbAllTheWay() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    JoystickButton waitButton = RobotContainer.coDriverA;
    addCommands(
        parallel(
          // new ShooterStop(0),
          // new ShooterStop(1),
          new ClimbSetSubsystemAlwaysActive(true),
          new IntakeStop(0),
          new IntakeStop(1)
        ),
        new ClimberHomePosition(),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        new ClimberToTop().withName("prepareToGrabBarTwo"),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        new ClimberToBottom().withName("pullRobotUpToBarTwo"),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        new FloppyArmUp(),//Constants.CLIMBER_FLOPPY_BAR_TWO_TO_THREE).withName("reachToBarThree"),
        new WaitCommand(1.5),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        // new ClimberToBarRelease(),
        new ClimberStop(false),
        new WaitCommand(1),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        new ClimberToTop().withName("grabBarThree"),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        new ClimberToBottom().withName("pullUpToBarThree"),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        new FloppyArmDown(),//Constants.CLIMBER_FLOPPY_BAR_THREE_TO_FOUR).withName("dropToGrabBarFour"),
        new WaitCommand(1.5),
        new SmartDashboardPutData("confirm climb", true),
        new WaitUntilCommand(waitButton::get),
        new SmartDashboardPutData("confirm climb", false),
        // new ClimberToBarRelease().withName("grabBarFour"),
        new ClimberStop(false),
        new WaitCommand(1),
        new FloppyArmUp()
    );
  }
}
