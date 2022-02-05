package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private VictorSPX intakeMotor;
	//TODO:add intake sensor, a digital sensor like in feeder used to detect ball. make acccessor too please
	
	/**
	 * Do not use
	 */
	@Deprecated
	public Intake() {
	}

	public Intake(int id) {
		intakeMotor = new VictorSPX(Constants.INTAKE_MOTORS[id]);
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
}