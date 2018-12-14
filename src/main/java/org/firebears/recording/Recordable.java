package org.firebears.recording;

import edu.wpi.first.wpilibj.AnalogGyro;

/**
 * Represents something connected to the robot that can be recorded, and
 * potentially set later. For instance, we could record the input to a
 * {@code SpeedController} or a {@code Joystick} and later set those values back
 * into the robot. We could record the values from an {@link AnalogGyro}, but it
 * would not make sense to set values into back to the gyro.
 */
public interface Recordable {

    /**
     * @return a unique label. Must not be null.
     */
    String getLabel();

    /**
     * @return the current value to be recorded.
     */
    double get();

    /**
     * Play a value back to the object. If the object is non-settable, nothing bad
     * should happen.
     * 
     * @param value new value to be set.
     */
    void set(double value);

    /**
     * Determine if this is a playable object. Non-settable objects may be recorded
     * for information purposes only. For instance, you may choose to record gyro or
     * accelerometer readings, even though you can't play those back explicitly.
     * 
     * @return whether values can be played back on the object.
     */
    default boolean isSettable() {
        return true;
    }

    /**
     * Request a change in whether this object is settable. For many recordable
     * objects that are inherently not settable, this will not do anything.
     * 
     * @param enable whether it should be possible to play values back.
     */
    public void setSettable(boolean enable);
}