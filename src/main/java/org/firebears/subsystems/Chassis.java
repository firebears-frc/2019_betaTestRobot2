package org.firebears.subsystems;

import java.util.Arrays;
import java.util.List;

import org.firebears.commands.ChassisDriveCommand;
import org.firebears.recording.Recordable;
import org.firebears.recording.RecordingFactory.SpeedControllerRecordable;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem that contains the drive wheels for the robot.
 */
public class Chassis extends Subsystem {

	private final WPI_TalonSRX frontLeft;
	private final WPI_TalonSRX rearLeft;
	private final SpeedControllerGroup leftMotors;
	private final WPI_TalonSRX frontRight;
	private final WPI_TalonSRX rearRight;
	private final SpeedControllerGroup rightMotors;
	private final DifferentialDrive robotDrive;

	private boolean DEBUG;
	private boolean brakeMode;

	public static final int PID_IDX = 0;
	public static final double ENCODER_TICKS_PER_INCH = 52.6;
	public static final double WHEEL_BASE_IN_INCHES = 20.0;

	public Chassis() {
		Preferences config = Preferences.getInstance();

		frontLeft = new WPI_TalonSRX(config.getInt("chassis.frontLeft.canID", 2));
		rearLeft = new WPI_TalonSRX(config.getInt("chassis.rearLeft.canID", 3));
		frontRight = new WPI_TalonSRX(config.getInt("chassis.frontRight.canID", 4));
		rearRight = new WPI_TalonSRX(config.getInt("chassis.rearRight.canID", 5));

		leftMotors = new SpeedControllerGroup(frontLeft, rearLeft);
		rightMotors = new SpeedControllerGroup(frontRight, rearRight);

		robotDrive = new DifferentialDrive(leftMotors, rightMotors);
		addChild(robotDrive);
		robotDrive.setSafetyEnabled(true);
		robotDrive.setExpiration(0.1);
		robotDrive.setMaxOutput(1.0);
		setBrakeMode(false);

		DEBUG = config.getBoolean("debug", false);
	}

	public List<Recordable> getRecordables() {
		// Preferences config = Preferences.getInstance();
		// double motionP = config.getDouble("chassis.motion.p", 1.0);
		// double motionI = config.getDouble("chassis.motion.i", 0.0);
		// double motionD = config.getDouble("chassis.motion.d", 0.0);
		// PIDSource leftSource = new PIDSource(){
		// @Override
		// public void setPIDSourceType(PIDSourceType pidSource) {
		// }
		// @Override
		// public double pidGet() {
		// return inchesTraveledLeft();
		// }
		// @Override
		// public PIDSourceType getPIDSourceType() {
		// return PIDSourceType.kDisplacement;
		// }
		// };
		// PIDSource rightSource = new PIDSource(){
		// @Override
		// public void setPIDSourceType(PIDSourceType pidSource) {
		// }
		// @Override
		// public double pidGet() {
		// return inchesTraveledRight();
		// }
		// @Override
		// public PIDSourceType getPIDSourceType() {
		// return PIDSourceType.kDisplacement;
		// }
		// };

		// return Arrays.asList(
		// new PIDRecordable(motionP, motionI, motionD, leftSource, leftMotors,
		// "leftDistance"),
		// new PIDRecordable(motionP, motionI, motionD, rightSource, rightMotors,
		// "rightDistance") );

		return Arrays.asList(new SpeedControllerRecordable(leftMotors, "leftSpeed"),
				new SpeedControllerRecordable(rightMotors, "rightSpeed"));
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ChassisDriveCommand());
	}

	public void setBrakeMode(boolean brakeMode) {
		frontLeft.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		rearLeft.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		frontRight.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		rearRight.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		this.brakeMode = brakeMode;
	}

	public boolean getBrakeMode() {
		return this.brakeMode;
	}

	/**
	 * Drive the robot.
	 * 
	 * @param speed    Amount of forward motion, in the range of -1.0 to 1.0.
	 * @param rotation Amount of angular rotation, in the range of -1.0 to 1.0.
	 */
	public void driveRobot(double speed, double rotation) {
		robotDrive.arcadeDrive(-1 * speed, rotation);
	}

	/**
	 * Drive the robot in "tankDrive" format.
	 * 
	 * @param leftSpeed  Speed for the left motor, in the range of -1.0 to 1.0.
	 * @param rightSpeed Speed for the left motor, in the range of -1.0 to 1.0.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed) {
		robotDrive.tankDrive(leftSpeed, rightSpeed);
	}

	/**
	 * @return Distance traveled in inches, since the robot was started.
	 */
	public double inchesTraveled() {
		return (inchesTraveledLeft() + inchesTraveledRight()) / 2;
	}

	/**
	 * @return Distance traveled by the left wheels in inches .
	 */
	public double inchesTraveledLeft() {
		return frontLeft.getSelectedSensorPosition(PID_IDX) / ENCODER_TICKS_PER_INCH;
	}

	/**
	 * @return Distance traveled by the right wheels in inches.
	 */
	public double inchesTraveledRight() {
		return frontRight.getSelectedSensorPosition(PID_IDX) / ENCODER_TICKS_PER_INCH;
	}

	int count = 0;
	long prevTime = System.currentTimeMillis();
	double prevDistance = 0.0, prevSpeed = 0.0, prevAcceleration = 0.0, prevJerk = 0.0;
	double maxSpeed = 0.0, maxAcceleration = 0.0, maxJerk = 0.0;

	@Override
	public void periodic() {
		if (DEBUG && ++count % 10 == 0) {
			double speed = 0.0, acceleration = 0.0, jerk = 0.0;
			long timeDiff = System.currentTimeMillis() - prevTime;
			speed = Math.abs(inchesTraveled() - prevDistance) / timeDiff;
			acceleration = Math.abs(speed - prevSpeed) / timeDiff;
			jerk = Math.abs(jerk - prevJerk) / timeDiff;
			SmartDashboard.putNumber("Speed", speed);
			SmartDashboard.putNumber("Acceleration", acceleration);
			SmartDashboard.putNumber("Jerk", jerk);
			if (speed > maxSpeed || acceleration > maxAcceleration || jerk > maxJerk) {
				maxSpeed = Math.max(maxSpeed, speed);
				maxAcceleration = Math.max(maxAcceleration, acceleration);
				maxJerk = Math.max(maxJerk, jerk);
				System.out.printf("Chassis: %3.1f/%3.1f   %3.1f/%3.1f   %3.1f/%3.1f %n", 
				    speed, maxSpeed, acceleration, maxAcceleration, jerk, maxJerk);
			}
			prevTime = System.currentTimeMillis();
			prevDistance = inchesTraveled();
			prevSpeed = speed;
			prevAcceleration = acceleration;
			prevJerk = jerk;
		}
	}
}
