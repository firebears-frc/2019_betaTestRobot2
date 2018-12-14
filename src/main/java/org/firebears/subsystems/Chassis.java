package org.firebears.subsystems;

import java.util.Arrays;
import java.util.List;

import org.firebears.commands.ChassisDriveCommand;
import org.firebears.recording.Recordable;
import org.firebears.recording.RecordingFactory.SpeedControllerRecordable;
import org.firebears.recording.RecordingFactory.PIDRecordable;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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

	public static final int PID_IDX = 0;
	public static final double ENCODER_TICKS_PER_FOOT = 52.6 / 12.0;
	public static final double WHEEL_BASE_IN_FEET = 20.0 / 12.0;
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
	}

	public List<Recordable> getRecordables() {
		// Preferences config = Preferences.getInstance();
		// double motionP = config.getDouble("chassis.motion.p", 1.0);
		// double motionI = config.getDouble("chassis.motion.i", 0.0);
		// double motionD = config.getDouble("chassis.motion.d", 0.0);
		// PIDSource leftSource = new PIDSource(){
		// 	@Override
		// 	public void setPIDSourceType(PIDSourceType pidSource) {
		// 	}
		// 	@Override
		// 	public double pidGet() {
		// 		return feetTraveledLeft();
		// 	}
		// 	@Override
		// 	public PIDSourceType getPIDSourceType() {
		// 		return PIDSourceType.kDisplacement;
		// 	}
		// };
		// PIDSource rightSource = new PIDSource(){
		// 	@Override
		// 	public void setPIDSourceType(PIDSourceType pidSource) {
		// 	}
		// 	@Override
		// 	public double pidGet() {
		// 		return feetTraveledRight();
		// 	}
		// 	@Override
		// 	public PIDSourceType getPIDSourceType() {
		// 		return PIDSourceType.kDisplacement;
		// 	}
		// };

		// return Arrays.asList(
		// 	new PIDRecordable(motionP, motionI, motionD, leftSource, leftMotors, "leftDistance"),
		// 	new PIDRecordable(motionP, motionI, motionD, rightSource, rightMotors, "rightDistance") );

		return Arrays.asList(
			new SpeedControllerRecordable(leftMotors, "leftSpeed"),
			new SpeedControllerRecordable(rightMotors, "rightSpeed") );
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ChassisDriveCommand());
	}

	/**
	 * Drive the robot.
	 * 
	 * @param speed    Amount of forward motion, in the range of -1.0 to 1.0.
	 * @param rotation Amount of angular rotation, in the range of -1.0 to 1.0.
	 */
	public void driveRobot(double speed, double rotation) {
		robotDrive.arcadeDrive(speed, rotation);
	}
	
	/**
	 * Drive the robot in "tankDrive" format.
	 * 
	 * @param leftSpeed Speed for the left motor, in the range of -1.0 to 1.0.
	 * @param rightSpeed Speed for the left motor, in the range of -1.0 to 1.0.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed) {
		robotDrive.tankDrive(leftSpeed, rightSpeed);
	}

	/**
	 * @return Distance traveled in feet, since the robot was started.
	 */
	public double feetTraveled() {
		return (feetTraveledLeft() + feetTraveledRight()) / 2;
	}
	
	/**
	 * @return Distance traveled by the left wheels in feet.
	 */
	public double feetTraveledLeft() {
		return frontLeft.getSelectedSensorPosition(PID_IDX) / ENCODER_TICKS_PER_FOOT;
	}

	/**
	 * @return Distance traveled by the right wheels in feet.
	 */
	public double feetTraveledRight() {
		return frontRight.getSelectedSensorPosition(PID_IDX) / ENCODER_TICKS_PER_FOOT;
	}


	@Override
	public void periodic() {
		// Put code here to be run every loop
	}
}
