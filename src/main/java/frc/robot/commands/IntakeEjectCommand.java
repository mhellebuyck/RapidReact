// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
//import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeEjectCommand extends CommandBase {
  private IntakeSubsystem _intake;

  /** Creates a new IntakeEjectCommand. */
  public IntakeEjectCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _intake = intake;
    // addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Intentional wrong direction because intake and outtake are reversed
    _intake.outtake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
