// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrees extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private double timeoutSeconds;
  private Timer safetyTimer = new Timer();

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double degrees, Drivetrain drive, double timeoutSeconds) {
    m_degrees = degrees;
    m_drive = drive;
    this.timeoutSeconds = timeoutSeconds;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.stop();
    m_drive.turnRelativeDegrees(m_degrees);
    safetyTimer.reset();
    safetyTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.turnToTargetHeading();
    // m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setTargetHeadingToCurrentHeading();
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = m_drive.getIsAtTargetHeading();

    if (safetyTimer.get() >= timeoutSeconds) {
        isFinished = true;
    }
    
    return isFinished;
  }
}
