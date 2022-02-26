package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private DoubleSolenoid intakeSolenoid;
	private TalonSRX intakeMotor;
	private DigitalInput intakeSensor;
	private boolean doPeriodic = false;
	private int id;

	public enum IntakeSolenoidPosition {
		kUp,
		kDown
	}

	/**
	 * Do not use
	 */
	@Deprecated
	public Intake() {
	}

	public Intake(int id) {
		intakeMotor = new TalonSRX(Constants.INTAKE_MOTORS[id]);
		if (id == 0) {
			intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.DROP_SOLENOID_REVERSE, Constants.DROP_SOLENOID_FORWARD);
		}
		intakeSensor = new DigitalInput(Constants.INTAKE_BALL_DETECTORS[id]);
		intakeMotor.setInverted(id == 0);
		intakeMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor.enableVoltageCompensation(true);
		intakeMotor.configVoltageCompSaturation(12.5);
		this.id = id;
		doPeriodic = true;

	}

	public void setPneumaticPosition(IntakeSolenoidPosition solenoidPosition) {
		if (id == 1) {
			return;
		}
        switch (solenoidPosition) {
            case kUp:
                intakeSolenoid.set(Value.kForward);
                break;
            case kDown:
                intakeSolenoid.set(Value.kReverse);
                break;
        }
	}

	public void periodic() {
		if (doPeriodic) {
			SmartDashboard.putBoolean("Intake Sensor " + (id == 0 ? "Left" : "Right"), getIntakeSensor());
		}
	}

	public void intake() {
		intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_INTAKE_SPEED);
		if (id == 0) {
			setPneumaticPosition(IntakeSolenoidPosition.kDown);
		}
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