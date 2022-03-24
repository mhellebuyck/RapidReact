// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubLift;

public class Shoot extends CommandBase {
  /** Creates a new RaiseLift. */
  private double m_duration;
  private double m_speed;
  private long m_startTime;
  private SubLift subsystem;
 

  public Shoot(SubLift lift, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = lift;
    addRequirements(subsystem);
    setM_speed(speed);
    setM_duration(time * 1000);

  }

  public long getM_startTime() {
    return m_startTime;
  }

  public void setM_startTime(long m_startTime) {
    this.m_startTime = m_startTime;
  }

  public double getM_speed() {
    return m_speed;
  }

  public void setM_speed(double m_speed) {
    this.m_speed = m_speed;
  }

  public double getM_duration() {
    return m_duration;
  }

  public void setM_duration(double m_duration) {
    this.m_duration = m_duration;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.liftRaise.set(0.50);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.StopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
