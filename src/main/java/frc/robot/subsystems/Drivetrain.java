// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.RomiGyro;
import frc.robot.RobotMap.PWM;

public class Drivetrain extends SubsystemBase {
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(11, 11, 11);
  Spark leftDrive = new Spark(PWM.DRIVETRAIN_LEFT);
  Spark rightDrive = new Spark(PWM.DRIVETRAIN_RIGHT);
  public Robot robot;
  // public final RomiGyro gyro = new RomiGyro();
  public ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  double targetHeading = 0;
  Timer turnTimer = new Timer();
  
  public Drivetrain(Robot robot) {
      this.robot = robot;
      System.out.println("Created Drivetrain object.");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetHeadingToCurrentHeading() {
    this.targetHeading =  gyro.getAngle(); // gyro.getOrientation();
  }

  public void maintainTurningTimer(boolean userIsTurning) {
    boolean timerWithinGracePeriod = turnTimer.get() < Constants.TURN_TIMER_TURNING_GRACE_PERIOD;
    boolean adjustHeadingDueToTimer = turnTimer.get() > 0 && timerWithinGracePeriod;

    if (userIsTurning) {
      turnTimer.reset();
      turnTimer.start();
    }

    if (userIsTurning || adjustHeadingDueToTimer) {
      targetHeading = gyro.getAngle();//.getOrientation();
    }

    if (turnTimer.get() >= Constants.TURN_TIMER_TURNING_GRACE_PERIOD) {
      turnTimer.stop();
      turnTimer.reset();
    }

    System.out.println("Turn timer: " + turnTimer.get());
  }

  public void moveArcadeLinear(double translation, double rotation) {
    System.out.println("Moving arcade. trans: " + translation + " rotation: " + rotation);
    leftDrive.setInverted(true);

    translation = applyDeadband(translation);
    rotation = applyDeadband(rotation);

    double leftMotorOutput;
    double rightMotorOutput;
    double maxInput = Math.copySign(Math.max(Math.abs(translation), Math.abs(rotation)), translation);

    if (translation >= 0.0) {
      // First quadrant, else second quadrant
      if (rotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = translation - rotation;
      } else {
        leftMotorOutput = translation + rotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (rotation >= 0.0) {
        leftMotorOutput = translation + rotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = translation - rotation;
      }
    }

    boolean userIsTurning = Math.abs(rotation) > Constants.JOYSTICK_DEADBAND;
    this.maintainTurningTimer(userIsTurning);

    double gyroAdjust = this.getGyroAdjustment();

    //System.out.println("Moving. TH: " + this.targetHeading + " CH: " + gyro.getOrientation() + " GA: " + gyroAdjust);

    leftMotorOutput += gyroAdjust;
    rightMotorOutput -= gyroAdjust;

    double leftVolts = convertPowerToVoltage(leftMotorOutput);
    double rightVolts = convertPowerToVoltage(rightMotorOutput);
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
  }

  /**
   * Set the speeds of the left and right sides of the robot.
   * 
   * We use 2 motors on each side, but we use a PWM splitter going from one port on the RoboRIO to 2 seperate Rev Robotics Spark motor controllers. Hence why we are able to have just the 2 controllers.
   * 
   * @param leftPower
   * @param rightPower
   */
  public void moveLinear(double leftPower, double rightPower) {
    leftDrive.setInverted(true);

    leftPower = applyDeadband(leftPower);
    rightPower = applyDeadband(rightPower);

    // TODO: Implement Exponential Drive.
    // leftDrive.setVoltage((feedforward.calculate(leftAmmount, leftAmmount)));
    // rightDrive.setVoltage((feedforward.calculate(rightAmmount, rightAmmount)));

    double netTurn = leftPower - rightPower;
    boolean userIsTurning = Math.abs(netTurn) > Constants.DRIVER_IS_TURNING_THRESHOLD;
    this.maintainTurningTimer(userIsTurning);

    double gyroAdjust = this.getGyroAdjustment();

    System.out.println("Moving. TH: " + this.targetHeading + " CH: " + gyro.getAngle() + " GA: " + gyroAdjust);

    leftPower += gyroAdjust;
    rightPower -= gyroAdjust;

    double leftVolts = convertPowerToVoltage(leftPower);
    double rightVolts = convertPowerToVoltage(rightPower);
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
  }

  protected double applyDeadband(double value) {
    double deadband = Constants.JOYSTICK_DEADBAND;
    double output = value;

    if (Math.abs(value) < deadband) {
      output = 0;
    }

    return output;
  }

  private double convertPowerToVoltage(double power) {
    double voltage = Constants.MAX_VOLTAGE * power;
    return voltage;
  }

  private double getGyroAdjustment() {
    double headingDifference = gyro.getAngle() - targetHeading;
    headingDifference %= 360;

		if (headingDifference < -180) {
      headingDifference += 360;
    }
      
    if (headingDifference > 180) {
      headingDifference -= 360;
    }

		double gyroAdjust = headingDifference * Constants.GYRO_ADJUST_SCALE_COEFFICIENT;

		return gyroAdjust;
  }

  public void turnRelativeDegrees(double degrees) {
    this.targetHeading += degrees;
  }

  /**
   * Stop all drive motors on the robot.
   */
  public void stop() {
      leftDrive.stopMotor();
      rightDrive.stopMotor();
  }

  public boolean getIsAtTargetHeading() {
    double headingDifference = Math.abs(this.targetHeading - this.gyro.getAngle());
    boolean isWithinHeadingBuffer = headingDifference < Constants.IS_AT_TARGET_HEADING_BUFFER_DEGREES;

    return isWithinHeadingBuffer;
  }

  public void turnToTargetHeading() {
    this.moveLinear(0, 0);
  }
}