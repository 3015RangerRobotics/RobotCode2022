package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
	public TalonFX shooter;
	public boolean doPeriodic = false;
	public double lastSetpoint = 0;
	public double speed;
	private double id;

	/**
	 * Do not use
	 */
	@Deprecated
	public Shooter() {
	}

	public Shooter(int id) {
		shooter = new TalonFX(Constants.SHOOTER_MOTORS[id]);

		shooter.configFactoryDefault();

		shooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
		shooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10);

		shooter.setNeutralMode(NeutralMode.Brake);

		shooter.enableVoltageCompensation(true);
		shooter.configVoltageCompSaturation(12.5);

		shooter.setInverted(id == 0);
		shooter.setSelectedSensorPosition(0);
		shooter.setSensorPhase(false);

		shooter.config_kP(0, Constants.SHOOTER_P);
		shooter.config_kI(0, Constants.SHOOTER_I);
		shooter.config_kD(0, Constants.SHOOTER_D);
		shooter.config_kF(0, Constants.SHOOTER_F);

		shooter.config_kP(1, Constants.SHOOTER_SHOOT_P);
		shooter.config_kI(1, Constants.SHOOTER_SHOOT_I);
		shooter.config_kD(1, Constants.SHOOTER_SHOOT_D);
		shooter.config_kF(1, Constants.SHOOTER_SHOOT_F);

		doPeriodic = true;
		this.id = id;
	}

	@Override
	public void periodic() {
		if (doPeriodic) {
			SmartDashboard.putNumber((id == 0 ? "Left " : "Right ") + "Shooter RPM", getRPM());
			// SmartDashboard.putNumber("PIDTarget", lastSetpoint);
			// SmartDashboard.putNumber("PIDActual", getRPM());
		}
	}

	/**
	 * @return The current RPM of the shooter wheel
	 */
	public double getRPM() {
		return (shooter.getSelectedSensorVelocity() * 10 * 60 / Constants.SHOOTER_PULSES_PER_ROTATION);
	}

	/**
	 * @param rpm The new RPM of the shooter wheel
	 */
	public void setRPM(double rpm) {
		shooter.set(ControlMode.Velocity, rpm / 10 / 60 * Constants.SHOOTER_PULSES_PER_ROTATION);
		lastSetpoint = rpm;
	}

	public void stop() {
		shooter.set(ControlMode.PercentOutput, 0);
		lastSetpoint = 0;
	}

	public void purgeShooter() {
		shooter.set(ControlMode.PercentOutput, -.2);
	}

	public boolean isPrimed(double speed) {
		return (Math.abs(speed - getRPM()) / speed < Constants.SHOOTER_TOLERANCE);
	}
}
