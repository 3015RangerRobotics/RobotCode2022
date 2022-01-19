package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
 
publc class Shooter extends Subsystem {
    public FalconFX shooter;

	//private DigitalInput ballSensor; ?will there be one on the robot?

    public Shooter() {
		shooter = new WPI_TalonFX(Constants.SHOOTER_MOTOR);

		shooter.configFactoryDefault();
		
		shooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
		shooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10);
		
		shooter.setNeutralMode(NeutralMode.Coast);
		
		shooter.enableVoltageCompensation(true);
		shooter.configVoltageCompSaturation(12.5);
		
		shooter.setInverted(false);
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

        //should a switch statement be used?
    }

    @Override
	public void periodic() {

        /**
	     * @return The RPM of the shooter wheel
	     */
	    public double getRPM() {
	    	return (shooter.getSelectedSensorVelocity() * 10 * 60 / Constants.SHOOTER_PULSES_PER_ROTATION);
	    }
    }
}