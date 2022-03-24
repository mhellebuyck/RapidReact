// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double MAX_VOLTAGE = 12;
	public static final double GYRO_ADJUST_SCALE_COEFFICIENT = .02;
	public static final double IS_AT_TARGET_HEADING_BUFFER_DEGREES = 3;
	public static final double JOYSTICK_DEADBAND = 0.15;
	public static final double TURN_TIMER_TURNING_GRACE_PERIOD = 0.5;
	public static final double DRIVER_IS_TURNING_THRESHOLD = 0.3;
	public static boolean ARCADE_DRIVE = true;
	public static final double INTAKE_COLLECT_POWER = 1;
    public static final double INTAKE_PUSH_POWER = -1;

}
