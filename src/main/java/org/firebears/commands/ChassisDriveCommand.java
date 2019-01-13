
package org.firebears.commands;

import org.firebears.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Default command to drive the robot chassis using the Joystick.
 */
public class ChassisDriveCommand extends Command {
    final int speedAxis;
    final int rotateAxis;

    public ChassisDriveCommand() {
        requires(Robot.chassis);
        Preferences config = Preferences.getInstance();
        speedAxis = config.getInt("joystick.speedAxis", 1);
        rotateAxis = config.getInt("joystick.rotateAxis", 0);
    }

    @Override
    protected void execute() {
        double speed = Robot.oi.getJoystick().getRawAxis(speedAxis);
        double rotation = Robot.oi.getJoystick().getRawAxis(rotateAxis);
        Robot.chassis.driveRobot(speed, rotation);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
