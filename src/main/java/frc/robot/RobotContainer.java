// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static Drive drive;
  public static Intake[] intake;
  public static Feeder[] feeder;
  public static Shooter[] shooter;
  public static Hood[] hood;
  public static Climber climber;
  public static Compressor compressor;

  private static final XboxController driver = new XboxController(0);
  private static final XboxController coDriver = new XboxController(1);

  private static final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
  private static final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
  private static final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
  private static final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
  private static final JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private static final JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private static final DPadButton driverDUp = new DPadButton(driver, DPadButton.Value.kDPadUp);
  private static final DPadButton driverDDown = new DPadButton(driver, DPadButton.Value.kDPadDown);
  private static final DPadButton driverDLeft = new DPadButton(driver, DPadButton.Value.kDPadLeft);
  private static final DPadButton driverDRight = new DPadButton(driver, DPadButton.Value.kDPadRight);
  private static final TriggerButton driverLT = new TriggerButton(driver, true);
  private static final TriggerButton driverRT = new TriggerButton(driver, false);
  private static final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
  private static final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);

  private static final JoystickButton coDriverA = new JoystickButton(coDriver, XboxController.Button.kA.value);
  private static final JoystickButton coDriverB = new JoystickButton(coDriver, XboxController.Button.kB.value);
  private static final JoystickButton coDriverX = new JoystickButton(coDriver, XboxController.Button.kX.value);
  private static final JoystickButton coDriverY = new JoystickButton(coDriver, XboxController.Button.kY.value);
  private static final JoystickButton coDriverLB = new JoystickButton(coDriver,
      XboxController.Button.kLeftBumper.value);
  private static final JoystickButton coDriverRB = new JoystickButton(coDriver,
      XboxController.Button.kRightBumper.value);
  private static final DPadButton coDriverDUp = new DPadButton(coDriver, DPadButton.Value.kDPadUp);
  private static final DPadButton coDriverDDown = new DPadButton(coDriver, DPadButton.Value.kDPadDown);
  private static final DPadButton coDriverDLeft = new DPadButton(coDriver, DPadButton.Value.kDPadLeft);
  private static final DPadButton coDriverDRight = new DPadButton(coDriver, DPadButton.Value.kDPadRight);
  private static final TriggerButton coDriverLT = new TriggerButton(coDriver, true);
  private static final TriggerButton coDriverRT = new TriggerButton(coDriver, false);
  private static final JoystickButton coDriverStart = new JoystickButton(coDriver, XboxController.Button.kStart.value);
  private static final JoystickButton coDriverBack = new JoystickButton(coDriver, XboxController.Button.kBack.value);

  private static final Timer timer = new Timer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new Drive();
    climber = new Climber();
    compressor = new Compressor();
    for (int i = 0; i < 2; i++) {
      intake[i] = new Intake(i);
      feeder[i] = new Feeder(i);
      shooter[i] = new Shooter(i);
      hood[i] = new Hood(i);
    }

    drive.setDefaultCommand(new DriveWithGamepad());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public static double getDriverLeftTrigger() {
    return driver.getLeftTriggerAxis();
  }

  public static double getDriverRightTrigger() {
    return driver.getRightTriggerAxis();
  }

  public static double getDriverLeftStickX() {
    return driver.getLeftX();
  }

  public static double getDriverLeftStickY() {
    return driver.getLeftY();
  }

  public static double getDriverRightStickX() {
    return driver.getRightX();
  }

  public static double getDriverRightStickY() {
    return driver.getRightY();
  }

  private static class TriggerButton extends Trigger {
    boolean isLeftTrigger;
    XboxController controller;

    public TriggerButton(XboxController controller, boolean isLeftTrigger) {
      this.isLeftTrigger = isLeftTrigger;
      this.controller = controller;
    }

    @Override
    public boolean get() {
      if (isLeftTrigger)
        return controller.getLeftTriggerAxis() >= 0.5;
      else {
        return controller.getRightTriggerAxis() >= 0.5;
      }
    }
  }

  private static class DPadButton extends Button {
    private final int dPadDegree;
    private final XboxController controller;

    public enum Value {
      kDPadRight, kDPadUpRight, kDPadUp, kDPadUpLeft, kDPadLeft, kDPadDownLeft, kDPadDown, kDPadDownRight,
    }

    /**
     * Creates a dpad button
     *
     * @param controller the controller to attach the button to
     * @param value      the dpad value
     */

    public DPadButton(XboxController controller, Value value) {
      this.controller = controller;
      switch (value) {
        case kDPadRight:
          this.dPadDegree = 90;
          break;
        case kDPadUpRight:
          this.dPadDegree = 45;
          break;
        case kDPadUp:
          this.dPadDegree = 0;
          break;
        case kDPadUpLeft:
          this.dPadDegree = 315;
          break;
        case kDPadLeft:
          this.dPadDegree = 270;
          break;
        case kDPadDownLeft:
          this.dPadDegree = 225;
          break;
        case kDPadDown:
          this.dPadDegree = 180;
          break;
        case kDPadDownRight:
          this.dPadDegree = 135;
          break;
        default:
          throw new AssertionError("Illegal value" + value);
      }
    }

    @Override
    public boolean get() {
      return controller.getPOV() == dPadDegree;
    }
  }

}