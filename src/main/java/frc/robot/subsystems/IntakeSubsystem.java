// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.subsystems;
 
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PWM;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
 
 
public class IntakeSubsystem extends SubsystemBase {
 
  Spark intakeMotor = new Spark(PWM.LIFT_RAISE);
  Spark liftPull = new Spark(PWM.LIFT_PULL);
 
  //DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_SOLENOID_EXTEND, RobotMap.INTAKE_SOLENOID_RETRACT);
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
       
    public IntakeSubsystem() {
        System.out.println("Created lift object.");
        //intakeSolenoid.set(kReverse);
    }
 
    /**
     * Set the speed of the motor.
     *
     * @param value
     */
    public void LiftBot() {
        liftPull.set(0.5);
    }
 
    /**
     * Set the speed of the motor.
     *
     * @param value
     */
    public void LowerBot() {
        liftPull.set(-0.5);
    }
 
    /**
     * Stop all motors for the lift.
     */
    public void outtake() {
        // intakeMotor.set(0.50);
        intakeMotor.set(Constants.INTAKE_COLLECT_POWER);
        //intakeSolenoid.set(kForward);
    }
 
    /**
     * Set the speed of the motor.
     *
     * @param value
     */
    public void intake() {
        //intakeMotor.set(-0.75);
        intakeMotor.set(Constants.INTAKE_PUSH_POWER);
        //intakeSolenoid.set(kReverse);
    }
 
    /**
     * Set the speed of the motor.
     *
     * @param value
     */
    public void StopLift() {
        liftPull.stopMotor();
    }
 
    /**
     * Stop all motors for the lift.
     */
    public void stop() {
        intakeMotor.stopMotor();
    }
}
