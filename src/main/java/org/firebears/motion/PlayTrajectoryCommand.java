package org.firebears.motion;

import java.io.File;

import org.firebears.Robot;
import org.firebears.subsystems.Chassis;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class PlayTrajectoryCommand extends Command {

	private final DistanceFollower leftFollower;
	private final DistanceFollower rightFollower;
	private double leftInitialDistance = 0.0;
	private double rightInitialDistance = 0.0;
	private final Preferences config;

	/**
	 * @param trajectory Trajectory of the center of the robot.
	 */
	public PlayTrajectoryCommand(Trajectory trajectory) {
		requires(Robot.chassis);
		config = Preferences.getInstance();
		TankModifier modifier = new TankModifier(trajectory).modify(Chassis.WHEEL_BASE_IN_FEET);
		Trajectory leftTrajectory = modifier.getLeftTrajectory();
		Trajectory rightTrajectory = modifier.getRightTrajectory();
		leftFollower = makeFollower(leftTrajectory);
		rightFollower = makeFollower(rightTrajectory);
	}

	/**
	 * @param leftTrajectory  Trajectory of the left wheel of the robot.
	 * @param rightTrajectory Trajectory of the right wheel of the robot.
	 */
	public PlayTrajectoryCommand(Trajectory leftTrajectory, Trajectory rightTrajectory) {
		requires(Robot.chassis);
		config = Preferences.getInstance();
		leftFollower = makeFollower(leftTrajectory);
		rightFollower = makeFollower(rightTrajectory);
	}

	/**
	 * @param leftTrajectoryFile  Full file path for CSV file giving the trajectory of the left wheel.
	 * @param rightTrajectoryFile Full file path for CSV file giving the trajectory of the right wheel.
	 */
	public PlayTrajectoryCommand(String leftTrajectoryFile, String rightTrajectoryFile) {
		requires(Robot.chassis);
		config = Preferences.getInstance();
		Trajectory leftTrajectory = Pathfinder.readFromCSV(new File(leftTrajectoryFile));
		Trajectory rightTrajectory = Pathfinder.readFromCSV(new File(rightTrajectoryFile));
		leftFollower = makeFollower(leftTrajectory);
		rightFollower = makeFollower(rightTrajectory);
	}

	private DistanceFollower makeFollower(Trajectory trajectory) {
		DistanceFollower follower = new DistanceFollower(trajectory);
		follower.configurePIDVA(config.getDouble("motion.follower.P", 1.0), config.getDouble("motion.follower.I", 0.0),
				config.getDouble("motion.follower.D", 0.0), config.getDouble("motion.follower.V", 0.0),
				config.getDouble("motion.follower.A", 0.0));
		return follower;
	}

	@Override
	protected void initialize() {
		leftFollower.reset();
		rightFollower.reset();
		leftInitialDistance = Robot.chassis.feetTraveledLeft();
		rightInitialDistance = Robot.chassis.feetTraveledRight();
	}

	@Override
	protected boolean isFinished() {
		return leftFollower.isFinished() || rightFollower.isFinished();
	}

	@Override
	protected void execute() {
		double leftDistance = Robot.chassis.feetTraveledLeft() - leftInitialDistance;
		double leftSpeed = leftFollower.calculate(leftDistance / 12);
		double rightDistance = Robot.chassis.feetTraveledRight() - rightInitialDistance;
		double rightSpeed = leftFollower.calculate(rightDistance / 12);
		Robot.chassis.tankDrive(leftSpeed, rightSpeed);
	}

	@Override
	protected void end() {
		Robot.chassis.driveRobot(0.0, 0.0);
	}

}
