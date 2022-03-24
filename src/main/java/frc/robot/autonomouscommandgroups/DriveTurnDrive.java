// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomouscommandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DriveTime;
import frc.robot.commands.IntakeEjectCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DoubleSolenoid;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTurnDrive extends SequentialCommandGroup {
  /** Creates a new DriveTurnDrive. */
  public DriveTurnDrive(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new DriveTime(.75,.6, drivetrain),
      //new IntakeSubsystem(Robot.intake)
      new IntakeEjectCommand(Robot.intake).withTimeout(2),
      new DriveTime(-.3, 4.5, drivetrain)
      //new TurnDegrees(-90, drivetrain, 3),
      //new Wait(drivetrain, 0.751),
      //new DriveTime(.75, 2, drivetrain)
      //new TurnDegrees(-30, drivetrain, 3),
      //new Wait(drivetrain, 0.75),
      //new DriveTime(-.75, 1.75, drivetrain),
      //new TurnDegrees(-60, drivetrain, 3),
      //new Wait(drivetrain,0.75),
      //new DriveTime(-.75, 0.3, drivetrain),
      //new TurnDegrees(-90, drivetrain, 3),
      //new Wait(drivetrain, 0.75),
      //new DriveTime(-.75, 1.63, drivetrain),
      //new Wait(drivetrain, 0.75),
      //new DriveTime(.75, 1.63, drivetrain),
      //new TurnDegrees(-90, drivetrain, 3),
      //new Wait(drivetrain, 0.75),
      //new DriveTime(.75, 1.1, drivetrain),
      //new TurnDegrees(-90, drivetrain, 3),
      //new Wait(drivetrain, 0.75),
      //new DriveTime(.75, 1.63, drivetrain),
      //new Wait(drivetrain, 0.75),
      //new DriveTime(-.75, 0.41, drivetrain),
      //new TurnDegrees(-90, drivetrain, 3),
      //new Wait(drivetrain,0.75),
      //new DriveTime(-.75, 0.7, drivetrain)
      //new TurnDegrees(90, drivetrain, 3),
      //new Wait(drivetrain,0.75),
      //new DriveTime(.75, 1.15, drivetrain),
      //new TurnDegrees(-55, drivetrain, 3),
      //new Wait(drivetrain,0.75),
      //new DriveTime(.75, 0.4, drivetrain)
    
      );
  }


}
