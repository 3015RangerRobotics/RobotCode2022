package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private TalonSRX intakeMotor;
	private DigitalInput intakeSensor;
	private boolean doPeriodic = false;
	private int id;

	/**
	 * Do not use
	 */
	@Deprecated
	public Intake() {
	}

	public Intake(int id) {
		intakeMotor = new TalonSRX(Constants.INTAKE_MOTORS[id]);
		intakeSensor = new DigitalInput(Constants.INTAKE_BALL_DETECTORS[id]);
		intakeMotor.setInverted(id == 0);
		this.id = id;
		doPeriodic = true;
	}

	public void periodic() {
		if (doPeriodic) {
			SmartDashboard.putBoolean("Intake Sensor " + (id == 0 ? "Left" : "Right"), getIntakeSensor());
		}
	}

	public void intake() {
		intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_INTAKE_SPEED);
	}

	public void purge() {
		intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_PURGE_SPEED);
	}

	public void stop() {
		intakeMotor.set(ControlMode.PercentOutput, 0);
	}

	public boolean getIntakeSensor() {
		return !intakeSensor.get();
	}
}