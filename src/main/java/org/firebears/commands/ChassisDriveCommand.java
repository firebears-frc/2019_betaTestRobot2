
package org.firebears.commands;

import org.firebears.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Default command to drive the robot chassis using the Joystick.
 */
public class ChassisDriveCommand extends Command {

    public ChassisDriveCommand() {
        requires(Robot.chassis);
    }

    @Override
    protected void execute() {
        double speed = Robot.oi.getJoystick().getY();
        double rotation = Robot.oi.getJoystick().getX();
        Robot.chassis.driveRobot(speed, rotation);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
