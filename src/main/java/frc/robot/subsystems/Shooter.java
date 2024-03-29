package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
	NetworkTable powerTable = NetworkTableInstance.getDefault().getTable("power");
	public TalonFX shooter;
	public boolean doPeriodic = false;
	public double lastSetpoint = 0;
	public double speed;
	private double id;
	private boolean override;
	boolean debug = false;

	/**
	 * Do not use
	 */
	@Deprecated
	public Shooter() {
	}

	public Shooter(int id) {
		shooter = new TalonFX(Constants.SHOOTER_MOTORS[id]);

		shooter.configFactoryDefault();

		shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 251);
        shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 247);
        shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 239);
        shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
        shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 229);

		shooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
		shooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

		shooter.setNeutralMode(NeutralMode.Coast);

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
			if (debug) {
				powerTable.getEntry((id == 0 ? "Left" : "Right") + " Shooter Current");
				SmartDashboard.putNumber((id == 0 ? "Left " : "Right ") + "Shooter RPM", getRPM());
				SmartDashboard.putBoolean((id == 0 ? "Left " : "Right ") + "Shooter Primed",
						isPrimed() && lastSetpoint != 0);
				if (id == 0) {
					int left = (isPrimed() && lastSetpoint != 0) ? 1 : 0;
					int right = SmartDashboard.getBoolean("Right Shooter Primed", false) ? 1 : 0;
					SmartDashboard.putNumber("Shooters ready", left + right);
				}
			}
			// SmartDashboard.putNumber("PIDTarget", lastSetpoint);
			// SmartDashboard.putNumber("PIDActual", getRPM());
			if (getRPM() < Constants.SHOOTER_REST_SPEED && lastSetpoint == 0 && !override) {
				/* Doing this, rather than setting the speed to the rest speed
				allows the shooter wheel to spin down naturally and still maintain
				a minimum speed, rather than forcefully slowing the motor down,
				wasting the battery, and shortening the lifespan of the motor. */
				setRPM(Constants.SHOOTER_REST_SPEED);
			}
		}
	}

	/**
	 * @return The current RPM of the shooter wheel
	 */
	public double getRPM() {
		return (shooter.getSelectedSensorVelocity() * 10 * 60 / Constants.SHOOTER_PULSES_PER_ROTATION);
	}

	public void setPercentOutput(double percent) {
		shooter.set(ControlMode.PercentOutput, percent);
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

	public boolean isPrimed() {
		return (Math.abs(lastSetpoint - getRPM()) / lastSetpoint < Constants.SHOOTER_TOLERANCE);
	}

	public boolean isInUse() {
		return (lastSetpoint != 0) && (lastSetpoint != Constants.SHOOTER_REST_SPEED);
	}

	public void setMinSpeedOverride(boolean override) {
		this.override = override;
	}

	public void setDebugMode(boolean debug) {
        this.debug = debug;
    }
}
