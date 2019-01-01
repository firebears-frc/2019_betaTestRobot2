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
	private final Preferences config;
	private boolean initialBrakeMode;

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

	private DistanceFollower makeFollower(Trajectory trajectory) {
		DistanceFollower follower = new DistanceFollower(trajectory);
		final double kp = config.getDouble("motion.follower.P", 1.0);
		final double ki = config.getDouble("motion.follower.I", 1.0);
		final double kd = config.getDouble("motion.follower.D", 1.0);
		final double kv = config.getDouble("motion.follower.V", 1.0);
		final double ka = config.getDouble("motion.follower.A", 1.0);
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
		Robot.chassis.setBrakeMode(false);
	}

	@Override
	protected boolean isFinished() {
		return leftFollower.isFinished() || rightFollower.isFinished();
	}

	@Override
	protected void execute() {
		double leftDistance = Robot.chassis.inchesTraveledLeft() - leftInitialDistance;
		double leftSpeed = leftFollower.calculate(leftDistance / 12);
		double rightDistance = Robot.chassis.inchesTraveledRight() - rightInitialDistance;
		double rightSpeed = leftFollower.calculate(rightDistance / 12);
		Robot.chassis.tankDrive(leftSpeed, rightSpeed);
	}

	@Override
	protected void end() {
		Robot.chassis.tankDrive(0.0, 0.0);
		Robot.chassis.setBrakeMode(initialBrakeMode);
	}

}
