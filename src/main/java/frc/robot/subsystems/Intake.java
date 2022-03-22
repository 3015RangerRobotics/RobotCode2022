package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	private DoubleSolenoid intakeSolenoid;
	NetworkTable powerTable = NetworkTableInstance.getDefault().getTable("power");
	private TalonSRX intakeMotor;
	private DigitalInput intakeSensor;
	private boolean doPeriodic = false;
	private boolean intakeDown = false;
	private boolean overridePneumatic = false;
	private Timer timer;
	private int id;

	private ColorSensorV3 colorSensor;

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
		if (id == 0) {
			// intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_REVERSE);
		}
		intakeMotor = new TalonSRX(Constants.INTAKE_MOTORS[id]);
		intakeSensor = new DigitalInput(Constants.INTAKE_BALL_DETECTORS[id]);
		intakeMotor.setInverted(id == 0);
		intakeMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor.enableVoltageCompensation(true);
		intakeMotor.configVoltageCompSaturation(12.5);
		this.id = id;
		doPeriodic = true;
		timer = new Timer();
		timer.reset();
		timer.start();

		colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
	}

	public void setPneumaticPosition(IntakeSolenoidPosition solenoidPosition) {
		if (id == 1) {
			return;
		}
        switch (solenoidPosition) {
            case kUp:
				if (!intakeDown) break;
				intakeDown = false;
				intakeSolenoid.set(Value.kReverse);
                break;
            case kDown:
				if (intakeDown) break;
				intakeDown = true;
                intakeSolenoid.set(Value.kForward);
                break;
        }
	}

	public void periodic() {
		if (doPeriodic) {
			if (id == 0 && !overridePneumatic) {
				if (timer.get() > 0.25) {
					setPneumaticPosition(IntakeSolenoidPosition.kUp);
				} else {
					setPneumaticPosition(IntakeSolenoidPosition.kDown);
				}
			}
			powerTable.getEntry((id == 0 ? "Left" : "Right") + "Intake Current").setDouble(intakeMotor.getSupplyCurrent());
			SmartDashboard.putBoolean("Intake Sensor " + (id == 0 ? "Left" : "Right"), getIntakeSensor());
			SmartDashboard.putBoolean("Intake Override Status", overridePneumatic);

			int red = colorSensor.getRed();
			int blue = colorSensor.getBlue();
			int proximity = colorSensor.getProximity();
			SmartDashboard.putNumber("red", red);
			SmartDashboard.putNumber("blue", blue);
			SmartDashboard.putNumber("proximity", proximity);
			SmartDashboard.putNumber("red - blue", red - blue);
			SmartDashboard.putNumber("blue - red", blue - red);
		}
	}
	
	public void intake() {
		intake(true);
	}

	public void intake(boolean dropIntake) {
		intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_INTAKE_SPEED);
		if (dropIntake) {
			timer.reset();
			timer.stop();
		}
	}

	public void purge() {
		intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_PURGE_SPEED);
	}

	public void stop() {
		stop(true);
	}

	public void stop(boolean raiseIntake) {
		intakeMotor.set(ControlMode.PercentOutput, 0);
		if (raiseIntake) {
			timer.reset();
			timer.start();
		}
	}

	public boolean getIntakeSensor() {
		return !intakeSensor.get();
	}

	public void setOverride(boolean override) {
		overridePneumatic = override;
	}

	public void test() {
		intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_TEST_SPEED);
	}

	public double getEncoderSpeed() {
		return intakeMotor.getSelectedSensorVelocity();
	}
}