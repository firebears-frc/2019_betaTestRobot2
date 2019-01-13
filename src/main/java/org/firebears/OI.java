package org.firebears;

import java.io.File;

import org.firebears.motion.PlayTrajectoryCommand;
import org.firebears.recording.PlayRecordingCommand;
import org.firebears.recording.StartRecordingCommand;
import org.firebears.recording.StopRecordingCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
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

	private XboxController joystick;

	public OI() {
		Preferences config = Preferences.getInstance();
		
		joystick = new XboxController(0);

		JoystickButton startRecordingButton = new JoystickButton(joystick, config.getInt("joystick.startRecordingButton", 8));
		Command startRecordingCommand = new StartRecordingCommand(Robot.recordingFactory);
		startRecordingButton.whenPressed(startRecordingCommand);

		JoystickButton stopRecordingButton = new JoystickButton(joystick, config.getInt("joystick.stopRecordingButton", 10));
		Command stopRecordingCommand = new StopRecordingCommand(Robot.recordingFactory);
		stopRecordingButton.whenPressed(stopRecordingCommand);

		JoystickButton playRecordingButton = new JoystickButton(joystick, config.getInt("joystick.playRecordingButton", 12));
		Command playRecordingCommand = new PlayRecordingCommand(Robot.recordingFactory);
		playRecordingButton.whenPressed(playRecordingCommand);
		
		JoystickButton playTrajectoryButton_Short = new JoystickButton(joystick, config.getInt("joystick.playTrajectoryButton_Short", 9));
		Trajectory Short_left  = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/Short.left.pf1.csv"));
		Trajectory Short_right = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/Short.right.pf1.csv"));
		Command playTrajectoryCommand_Short = new PlayTrajectoryCommand(Short_left, Short_right);
		playTrajectoryButton_Short.whenPressed(playTrajectoryCommand_Short);

		JoystickButton playTrajectoryButton_Straight = new JoystickButton(joystick, config.getInt("joystick.playTrajectoryButton_Straight", 11));
		Trajectory Straight_left  = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/Straight.left.pf1.csv"));
		Trajectory Straight_right = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/Straight.right.pf1.csv"));
		Command playTrajectoryCommand_Straight = new PlayTrajectoryCommand(Straight_left, Straight_right);
		playTrajectoryButton_Straight.whenPressed(playTrajectoryCommand_Straight);
	}

	public GenericHID getJoystick() {
		return joystick;
	}
}
