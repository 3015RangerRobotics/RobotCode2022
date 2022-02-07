package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private VictorSPX intakeMotor;
	private DigitalInput intakeSensor;

	/**
	 * Do not use
	 */
	@Deprecated
	public Intake() {
	}

	public Intake(int id) {
		intakeMotor = new VictorSPX(Constants.INTAKE_MOTORS[id]);
		//TODO: Construct the intakeSensor values are in Constants already
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
		return intakeSensor.get();
	}
}