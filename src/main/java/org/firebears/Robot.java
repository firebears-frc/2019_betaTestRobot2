package org.firebears;

import static org.firebears.util.Config.*;

import org.firebears.motion.TrajectoryFactory;
import org.firebears.recording.RecordingFactory;
import org.firebears.subsystems.Chassis;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project....
 */
public class Robot extends TimedRobot {

	Command autonomousCommand = null;

	public static OI oi;
	public static Chassis chassis;
	public static RecordingFactory recordingFactory;
	public static TrajectoryFactory trajectoryFactory;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		cleanAllPreferences();
		loadConfiguration("config.properties", "/home/lvuser/robot.properties", "/u/override.properties");
		printPreferences(System.out);

		recordingFactory = new RecordingFactory();

		chassis = new Chassis();
		recordingFactory.addAll(chassis.getRecordables());
		
		trajectoryFactory = new TrajectoryFactory();
		trajectoryFactory.init();
		
		oi = new OI();
	}

	/**
	 * This function is called when the disabled button is hit. You can use it to
	 * reset subsystems before shutting down.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// TODO:  select the autonomous command
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
}
