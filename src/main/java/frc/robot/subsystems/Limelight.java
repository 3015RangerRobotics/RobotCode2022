package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.ArrayList;

public class Limelight extends SubsystemBase {
	NetworkTable limelight;
	ArrayList<Double> avgDistance = new ArrayList<>();
	double users = 0;

	public enum LEDMode {
		PIPELINE(0),
		LED_OFF(1),
		LED_BLINK(2),
		LED_ON(3);

		private int m;

		LEDMode(int mode) {
			m = mode;
		}

		public int getMode() {
			return m;
		}
	}

	public enum StreamingMode {
		STANDARD(0),
		PIP_MAIN(1),
		PIP_SECONDARY(2);

		private int m;

		StreamingMode(int mode) {
			m = mode;
		}

		public int getMode() {
			return m;
		}
	}

	public enum CameraMode {
		VISION_PROCESSING(0),
		DRIVER_CAMERA(1);

		private int m;

		CameraMode(int mode) {
			m = mode;
		}

		public int getMode() {
			return m;
		}
	}

	/**
	 * Creates a new Limelight.
	 */
	public Limelight() {
		limelight = NetworkTableInstance.getDefault().getTable("limelight");

		setLEDMode(LEDMode.LED_OFF);
		setStreamingMode(StreamingMode.STANDARD);
	}

	@Override
	public void periodic() {
		// SmartDashboard output
		// avgDistance.add(getRobotToTargetDistance());
		// if(avgDistance.size() == 10){ //??
		// avgDistance.remove(0);
		// }
		SmartDashboard.putNumber("Target Distance", getRobotToTargetDistance());
		SmartDashboard.putNumber("Corrected Distance", getRobotCorrectedDistance());
		SmartDashboard.putNumber("Distance from LL", getDistanceFromLLPlane());
		double correctedDistance = getCorrectedDistanceFromLLPlane();
		SmartDashboard.putNumber("Corrected Distance from LL", correctedDistance);
		SmartDashboard.putNumber("Corrected Distance Inches", correctedDistance * 39.37);
		SmartDashboard.putNumber("Target Angle X", getTargetAngleX());
		SmartDashboard.putNumber("Corrected Angle X", getCorrectedAngleX());
		SmartDashboard.putNumber("Limelight Users", users);
		SmartDashboard.putNumber("Limelight RPM Target", getShooterSpeed());
		SmartDashboard.putNumber("Limelight Hood Target", getHoodPos());

		if (users > 0) {
			setLEDMode(LEDMode.LED_ON);
		} else if (users == 0) {
			setLEDMode(LEDMode.LED_OFF);
		} else { // users is negative, which should never happen
			setLEDMode(LEDMode.LED_BLINK);
		}
	}

	public void checkout() {
		users++;
	}

	public void uncheckout() {
		users--;
	}

	public double getAvgDistance() { // ??
		double total = 0;
		for (double d : avgDistance) {
			total += d;
		}
		return total / avgDistance.size();
	}

	/**
	 * @return if the limelight has a target
	 */
	public boolean hasTarget() {
		return limelight.getEntry("tv").getDouble(0) == 1;
	}

	/**
	 * @return The X angle to the target
	 */
	public double getTargetAngleX() {
		return limelight.getEntry("tx").getDouble(0);
	}

	// public double getCorrectedAngleX() {
	// if (Constants.LL_OFFSET == 0) {
	// return getTargetAngleX();
	// }
	// double angle = Math.toRadians(getTargetAngleX());
	// double distToTar = (getRobotToTargetDistance() +
	// Constants.LL_ROBOT_TO_TARGET);
	// double output = (distToTar * distToTar) + (Constants.LL_OFFSET *
	// Constants.LL_OFFSET) +
	// (2 * distToTar * Constants.LL_OFFSET * Math.sin(angle));
	// output = -1 * output / (2 * Constants.LL_OFFSET * Math.sqrt(output));
	// output = Math.asin(output);
	// return Math.toDegrees(output);
	// }

	// public double getCorrectedAngleX() {
	// double angle = Math.toRadians(90 - getTargetAngleX());
	// double targetDistance = getRobotToTargetDistance() +
	// Constants.LL_ROBOT_TO_TARGET;
	// double thirdSide = Math.pow(targetDistance, 2) +
	// Math.pow(Constants.LL_OFFSET, 2)
	// - (2 * targetDistance * Constants.LL_OFFSET * Math.cos(angle));
	// double correctedAngle = Math.asin(targetDistance * (Math.sin(angle) /
	// thirdSide));
	// return Math.toDegrees(correctedAngle);
	// }

	public double getShooterSpeed() {
		// double distance = (getCorrectedDistanceFromLLPlane() -
		// Constants.LL_GOAL_RADIUS)
		double distance = getDistanceInches();
		double lookupRPM = Constants.SHOOTER_LOOKUP_TABLE.lookupCeil(distance);
		SmartDashboard.putNumber("Lookup Distance Inches", distance);
		return lookupRPM + Constants.SHOOTER_LL_ADJUST;
	}

	public double getHoodPos() {
		double distance = getDistanceInches();
		double lookupAngle = Constants.HOOD_POSITION_TABLE.lookupFloor(distance);
		return lookupAngle;
	}

	public double getCorrectedAngleX() {
		double distance = getDistanceFromLLPlane();
		double correctedDistance = getCorrectedDistanceFromLLPlane();
		double angleX = getTargetAngleX();
		double sinAngle = Math.sin(Math.toRadians(90 - angleX));
		if (angleX > 0) {
			return -Math.toDegrees(Math.asin(sinAngle * distance / correctedDistance)) + 90;
		} else {
			return Math.toDegrees(Math.asin(sinAngle * distance / correctedDistance)) - 90;
		}
	}

	/**
	 * @return The Y angle to the target
	 */
	public double getTargetAngleY() {
		return limelight.getEntry("ty").getDouble(0);
	}

	/**
	 * @return The current latency from the limelight to the robot
	 */
	public double getLatency() {
		return (limelight.getEntry("tl").getDouble(0) + 11) / 1000.0;
	}

	/**
	 * @return The current active pipeline
	 */
	public int getPipeline() {
		return limelight.getEntry("getpipe").getNumber(0).intValue();
	}

	/**
	 * @return The solution to solvePnP
	 */
	public double[] get3DSolution() {
		return limelight.getEntry("camtran").getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
	}

	public double get3DDistance() {
		return Math.sqrt(getTranslationZ() * getTranslationZ() + getTranslationX() * getTranslationX());
	}

	public double getArea() {
		return limelight.getEntry("ta").getDouble(0);
	}

	/**
	 * @return The X distance to the target
	 */
	public double getTranslationX() {
		return get3DSolution()[0];
	}

	/**
	 * @return The Y distance to the target
	 */
	public double getTranslationY() {
		return get3DSolution()[1];
	}

	/**
	 * @return The Z distance to the target
	 */
	public double getTranslationZ() {
		return get3DSolution()[2];
	}

	/**
	 * @return Target pitch
	 */
	public double getRotationPitch() {
		return get3DSolution()[3];
	}

	/**
	 * @return Target yaw
	 */
	public double getRotationYaw() {
		return get3DSolution()[4];
	}

	/**
	 * @return Target roll
	 */
	public double getRotationRoll() {
		return get3DSolution()[5];
	}

	/**
	 * Set the LED mode
	 * 
	 * @param mode The mode to use
	 */
	public void setLEDMode(LEDMode mode) {
		limelight.getEntry("ledMode").setNumber(mode.getMode());
	}

	/**
	 * Set the camera mode
	 * 
	 * @param mode The mode to use
	 */
	public void setCameraMode(CameraMode mode) {
		limelight.getEntry("camMode").setNumber(mode.getMode());
	}

	/**
	 * Set the streaming mode
	 * 
	 * @param mode The mode to use
	 */
	public void setStreamingMode(StreamingMode mode) {
		limelight.getEntry("stream").setNumber(mode.getMode());
	}

	/**
	 * Set the processing pipeline
	 * 
	 * @param id The id of the pipeline to use
	 */
	public void setPipeline(int id) {
		limelight.getEntry("pipeline").setNumber(id);
	}

	public double getRobotToTargetDistance() {
		return ((Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT)
				/ Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + getTargetAngleY()))
				- Constants.LL_ROBOT_TO_TARGET);
	}

	public double getRobotCorrectedDistance() {
		return (Math.sqrt(((Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT) /
				Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + getTargetAngleY())))
				* ((Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT) /
						Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + getTargetAngleY())))
				- (Constants.LL_OFFSET * Constants.LL_OFFSET)) - Constants.LL_BACK_OFFSET
				- Constants.LL_ROBOT_TO_TARGET);
	}

	public double getDistanceFromLLPlane() {
		return (Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT)
				/ Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + getTargetAngleY())) + Constants.LL_GOAL_RADIUS;
	}

	public double getCorrectedDistanceFromLLPlane() {
		double distance = getDistanceFromLLPlane();
		return (Math.sqrt((Constants.LL_OFFSET * Constants.LL_OFFSET) + (distance * distance)
				- (2.0 * distance * Constants.LL_OFFSET * Math.sin(Math.toRadians(getTargetAngleX())))));
	}

	public double getDistanceInches() {
		double angle = Math.toRadians(getTargetAngleY() + Constants.LL_MOUNT_ANGLE);
		double linearDistance = (Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT) / Math.tan(angle);
		double adjustedDitance = linearDistance - Constants.LL_BACK_OFFSET - Constants.LL_RIM_TO_FENDER;
		return adjustedDitance * 39.37008;

	}

	/*
	 * public double getRobotToTargetDistance() {
	 * return (Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT) /
	 * Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + getTargetAngleY()));
	 * }
	 * 
	 * public double getShooterLaunchVelocity() {
	 * double g = 9.81;
	 * double y = Constants.LL_TARGET_HEIGHT;
	 * double x = getRobotToTargetDistance();
	 * double launchAngle = Constants.SHOOTER_LAUNCH_ANGLE; // Set to proper value
	 * double tanA = Math.tan(Math.toRadians(launchAngle));
	 * double upper = Math.sqrt(g) * Math.sqrt(x) * Math.sqrt(Math.pow(tanA, 2) +
	 * 1);
	 * double lower = Math.sqrt(2 * tanA - ((2 * y) / x));
	 * return Units.metersToFeet(upper / lower);
	 * }
	 */
}