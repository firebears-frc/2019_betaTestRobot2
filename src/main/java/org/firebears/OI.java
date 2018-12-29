package org.firebears;

import java.io.File;

import org.firebears.motion.PlayTrajectoryCommand;
import org.firebears.recording.PlayRecordingCommand;
import org.firebears.recording.StartRecordingCommand;
import org.firebears.recording.StopRecordingCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private Joystick joystick;

	public OI() {
		Preferences config = Preferences.getInstance();
		
		joystick = new Joystick(0);

		JoystickButton startRecordingButton = new JoystickButton(joystick, config.getInt("joystick.startRecordingButton", 8));
		Command startRecordingCommand = new StartRecordingCommand(Robot.recordingFactory);
		startRecordingButton.whenPressed(startRecordingCommand);

		JoystickButton stopRecordingButton = new JoystickButton(joystick, config.getInt("joystick.stopRecordingButton", 10));
		Command stopRecordingCommand = new StopRecordingCommand(Robot.recordingFactory);
		stopRecordingButton.whenPressed(stopRecordingCommand);

		JoystickButton playRecordingButton = new JoystickButton(joystick, config.getInt("joystick.playRecordingButton", 12));
		Command playRecordingCommand = new PlayRecordingCommand(Robot.recordingFactory);
		playRecordingButton.whenPressed(playRecordingCommand);
		
		JoystickButton playTrajectoryButton_leftScale = new JoystickButton(joystick, config.getInt("joystick.playTrajectoryButton_leftScale", 9));
		Trajectory leftScale_left = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/leftScale.left.pf1.csv"));
		Trajectory leftScale_right = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/leftScale.left.pf1.csv"));
		Command playTrajectoryCommand_leftScale = new PlayTrajectoryCommand(leftScale_left, leftScale_right);
		playTrajectoryButton_leftScale.whenPressed(playTrajectoryCommand_leftScale);

		JoystickButton playTrajectoryButton_leftSwitch = new JoystickButton(joystick, config.getInt("joystick.playTrajectoryButton_leftSwitch", 11));
		Trajectory leftSwitch_left = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/leftSwitch.left.pf1.csv"));
		Trajectory leftSwitch_right = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/leftSwitch.left.pf1.csv"));
		Command playTrajectoryCommand_leftSwitch = new PlayTrajectoryCommand(leftSwitch_left, leftSwitch_right);
		playTrajectoryButton_leftSwitch.whenPressed(playTrajectoryCommand_leftSwitch);
	}

	public Joystick getJoystick() {
		return joystick;
	}
}
