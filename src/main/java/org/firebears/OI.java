package org.firebears;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

import org.firebears.commands.*;
import org.firebears.motion.*;
import org.firebears.recording.*;
import org.firebears.subsystems.*;

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
		
		JoystickButton playTrajectoryButton = new JoystickButton(joystick, config.getInt("joystick.playTrajectoryButton", 11));
		Command playTrajectoryCommand = new PlayTrajectoryCommand(Robot.trajectoryFactory.trajectory1);
		playTrajectoryButton.whenPressed(playTrajectoryCommand);
	}

	public Joystick getJoystick() {
		return joystick;
	}
}
