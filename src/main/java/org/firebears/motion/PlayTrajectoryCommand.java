package org.firebears.motion;

import org.firebears.Robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

public class PlayTrajectoryCommand extends Command {

	private final DistanceFollower leftFollower;
	private final DistanceFollower rightFollower;
	private double leftInitialDistance = 0.0;
	private double rightInitialDistance = 0.0;
	private boolean initialBrakeMode;
	private final Preferences config;
	private final boolean DEBUG;

	/**
	 * @param leftTrajectory  Trajectory of the left wheel of the robot.
	 * @param rightTrajectory Trajectory of the right wheel of the robot.
	 */
	public PlayTrajectoryCommand(Trajectory leftTrajectory, Trajectory rightTrajectory) {
		requires(Robot.chassis);
		config = Preferences.getInstance();
		leftFollower = makeFollower(leftTrajectory);
		rightFollower = makeFollower(rightTrajectory);
		DEBUG = config.getBoolean("debug", false);
	}

	private DistanceFollower makeFollower(Trajectory trajectory) {
		DistanceFollower follower = new DistanceFollower(trajectory);
		final double kp = config.getDouble("motion.follower.P", 1.0);
		final double ki = config.getDouble("motion.follower.I", 0.0);
		final double kd = config.getDouble("motion.follower.D", 0.0);
		final double kv = 1.0 / config.getDouble("motion.maxVelocity", 10.0);
		final double ka = config.getDouble("motion.follower.A", 0.0);
		follower.configurePIDVA(kp, ki, kd, kv, ka);
		return follower;
	}

	@Override
	protected void initialize() {
		leftFollower.reset();
		rightFollower.reset();
		leftInitialDistance = Robot.chassis.inchesTraveledLeft();
		rightInitialDistance = Robot.chassis.inchesTraveledRight();
		initialBrakeMode = Robot.chassis.getBrakeMode();
		Robot.chassis.setBrakeMode(config.getBoolean("motion.brakeMode", true));
	}

	@Override
	protected boolean isFinished() {
		return leftFollower.isFinished() || rightFollower.isFinished();
	}

	@Override
	protected void execute() {
		double leftDistance = Robot.chassis.inchesTraveledLeft() - leftInitialDistance;
		double leftSpeed = leftFollower.calculate(leftDistance);
		double rightDistance = Robot.chassis.inchesTraveledRight() - rightInitialDistance;
		double rightSpeed = leftFollower.calculate(rightDistance);
		Robot.chassis.tankDrive(leftSpeed, rightSpeed);
		if (DEBUG) { 
			System.out.println("::: motion: (" + leftSpeed + ", " + rightSpeed + ")  [" + leftDistance + ", " + rightDistance + "]");  
		}
	}

	@Override
	protected void end() {
		Robot.chassis.tankDrive(0.0, 0.0);
		Robot.chassis.setBrakeMode(initialBrakeMode);
	}

}
