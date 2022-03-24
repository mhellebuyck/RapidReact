// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class SubLift extends SubsystemBase {
  /** Creates a new Lift. */
  public  Spark liftRaise = new Spark(RobotMap.PWM.LIFT_RAISE);
  public  Spark liftPull = new Spark(RobotMap.PWM.LIFT_PULL);
  
  
  public SubLift() {
    System.out.println("Created lift object.");

  }
   /**
     * Set the speed of the motor.
     * 
     * @param value
     */
    public void LiftBot() {
      liftPull.set(1);
    }

  /**
   * Set the speed of the motor.
   * 
   * @param value
   */
  public void LowerBot() {
      liftPull.set(-1);
  }

  /**
   * Stop all motors for the lift.
   */
  public void RaiseLift() {
      liftRaise.set(0.50);
  }

  /**
   * Set the speed of the motor.
   * 
   * @param value
   */
  public void LowerRaise() {
      liftRaise.set(-0.75);
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
  public void StopRaise() {
      liftRaise.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
        
   
}
