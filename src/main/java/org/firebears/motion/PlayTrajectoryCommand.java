package org.firebears.motion;

import org.firebears.Robot;
import org.firebears.subsystems.Chassis;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class PlayTrajectoryCommand extends Command {

	private final DistanceFollower leftFollower;
	private final DistanceFollower rightFollower;
	private double leftInitialDistance = 0.0;
	private double rightInitialDistance = 0.0;

	public static final double INCHES_PER_METER = 39.37;

	public PlayTrajectoryCommand(Trajectory trajectory) {
		requires(Robot.chassis);
		Preferences config = Preferences.getInstance();

		TankModifier modifier = new TankModifier(trajectory).modify(Chassis.WHEEL_BASE_IN_INCHES / INCHES_PER_METER);
		Trajectory leftTrajectory = modifier.getLeftTrajectory();
		Trajectory rightTrajectory = modifier.getRightTrajectory();
		leftFollower = new DistanceFollower(leftTrajectory);
		rightFollower = new DistanceFollower(rightTrajectory);

		leftFollower.configurePIDVA(
				config.getDouble("motion.follower.P", 1.0),
				config.getDouble("motion.follower.I", 0.0), 
				config.getDouble("motion.follower.D", 0.0),
				config.getDouble("motion.follower.V", 0.0), 
				config.getDouble("motion.follower.A", 0.0));
		rightFollower.configurePIDVA(
				config.getDouble("motion.follower.P", 1.0),
				config.getDouble("motion.follower.I", 0.0), 
				config.getDouble("motion.follower.D", 0.0),
				config.getDouble("motion.follower.V", 0.0), 
				config.getDouble("motion.follower.A", 0.0));
	}

	@Override
	protected void initialize() {
		leftFollower.reset();
		rightFollower.reset();
		leftInitialDistance = Robot.chassis.inchesTraveledLeft();
		rightInitialDistance = Robot.chassis.inchesTraveledRight();
	}

	@Override
	protected boolean isFinished() {
		return leftFollower.isFinished() || rightFollower.isFinished();
	}

	@Override
	protected void execute() {
		double leftDistance = Robot.chassis.inchesTraveledLeft() - leftInitialDistance;
		double leftSpeed = leftFollower.calculate(leftDistance / INCHES_PER_METER);
		double rightDistance = Robot.chassis.inchesTraveledRight() - rightInitialDistance;
		double rightSpeed = leftFollower.calculate(rightDistance / INCHES_PER_METER);
		Robot.chassis.tankDrive(leftSpeed, rightSpeed);
	}

	@Override
	protected void end() {
		Robot.chassis.driveRobot(0.0, 0.0);
	}

}
