package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.RobotMap.Controls;
import frc.robot.RobotMap.PWM;
//import edu.wpi.first.wpilibj.DoubleSolenoid;


//public class Shooter2 extends Subsystem{
    // put methods for controlling this subsystem
    //here. Call these from commands.
    //DoubleSolenoid pitchSolenoid = null;

   // public Shooter2() {
       // pitchSolenoid = new DoubleSolenoid(RobotMap.SHOOTER_PITCH_SOLENOID_DEPLOY , RobotMap.SHOOTER_PITCH_SOLENOID_RETRACT);
   // }

   // @Override
   // public void initDefaultCommand(m_shooter = new Shooter2();) {public static Shooter2 m_shooter = null;
        // set the default command for a subsystem here.
        //setDefualtCommand(new MySpecialCommand());
    //{
   // Robotmap1.Java(570425394); {26,5} AssigmentOperatorExspression
        //Solenoids
       // public static final int SHOOTER_PITCH_SOLENOID_DEPLOY = 0;
        // public static final int SHOOTER_PITCH_SOLENOID_RETRACT = 1;
    

//} 


public class Systems{

    

    public class ControlPanel {
        
        Spark controlPanel = new Spark(PWM.CONTROL_PANEL);

        ControlPanel() {
            System.out.println("Created ControlPanel object.");
        }

        /**
         * Set the speed of the motor.
         * 
         * @param value
         */
        public void Move(double value) {

            // TODO: Implement Exponential Drive.
            controlPanel.set(value);

        }

        /**
         * Stop all motors for the control panel.
         */
        public void Stop() {
            controlPanel.stopMotor();
        }
    }

    


    public class Shooter {
        
        Spark shooterTop = new Spark(PWM.SHOOTER_TOP);
        Spark shooterBottom = new Spark(PWM.SHOOTER_BOTTOM);
        Spark shooterIn = new Spark(PWM.SHOOTER_IN);

        Shooter() {
            System.out.println("Created Shooter object.");
        }

        public void raise() {
            shooterTop.set(1);
        }

        public void shoot(double intensity) {
            System.out.println("Intencity:" + intensity);
            shooterBottom.set(intensity);
        }

        public void pickUp() {
            shooterIn.set(1);
        }
        
        public void drop() {
            shooterIn.set(-1);
            shooterTop.set(-1);
        }

        public void stopPickUp() {
            shooterIn.set(0);
        }

        public void stopShoot() {
            shooterBottom.set(0);
        }

        public void stopRaise() {
            shooterTop.set(0);
        }
    }


    /**
     * The class for receiving input from the joysticks in Driver Station.
     * 
     * We use a dual Joystick configuration for more percise control of the robot. 
     */
    public class Input {
        Joystick left = new Joystick(0);
        Joystick right = new Joystick(1);

        Input() {
            System.out.println("Created Input object.");
        }
        
        public Double getLeftDrive() {
            return left.getRawAxis(Controls.LEFT_DRIVE_AXIS);
        }
        public Double getRightDrive() {
            // return left.getRawAxis(4);
            return right.getRawAxis(Controls.DRIVE_AXIS);
        }

        public boolean getControlPanelActive() {
            return (right.getRawButton(Controls.CONTROL_PANEL_RIGHT) || left.getRawButton(Controls.CONTROL_PANEL_LEFT)) ? true : false;
        }

        public boolean getShoot() {
            return right.getRawButton(Controls.SHOOTER_SHOOT);
        }

        public boolean getRaise() {
            return left.getRawButton(Controls.SHOOTER_RAISE);
        }

        public boolean getLower() {
            return left.getRawButton(Controls.SHOOTER_LOWER);
        }

        public boolean isDrivetrainLocked() {
            return right.getRawButton(Controls.DRIVETRAIN_LOCK);
        }

        public double shooterIntensity() {
            System.out.println((right.getThrottle() * .5 ) + .5);
            return (right.getThrottle() * .5 ) + .5;
        }

        public boolean liftClaw() {
            return left.getRawButton(Controls.RAISE_LIFT);
        }

        public boolean lowerClaw() {
            return left.getRawButton(Controls.LIFT_BOT);
        }

        public boolean liftBot() {
            return right.getRawButton(Controls.LIFT_BOT);
        }

        public boolean lowerBot() {
            return right.getRawButton(Controls.RAISE_LIFT);
        }
    }

}