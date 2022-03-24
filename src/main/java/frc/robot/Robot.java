// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.lang.reflect.Array;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
//import frc.robot.commands.TurnDegrees;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj2.TimedRobot;
//import edu.wpi.first.wpilibj.SPI;
// import frc.robot.OnBoardIO.ChannelMode;
import frc.robot.Systems.ControlPanel;
import frc.robot.Systems.Input;
import frc.robot.Systems.Shooter;
//import frc.robot.commands.DriveTime;
import frc.robot.autonomouscommandgroups.DriveTurnDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;




//public class Shooter2 extends Subsystem{
    // put methods for controlling this subsystem
    //here. Call these from commands.
  //  DoubleSolenoid pitchSolenoid = null;

    //public Shooter2() {
      //  pitchSolenoid = new DoubleSolenoid(RobotMap.SHOOTER_PITCH_SOLENOID_DEPLOY , RobotMap.SHOOTER_PITCH_SOLENOID_RETRACT);
    //}

    //@Override
    //public void initDefaultCommand(m_shooter = new Shooter2();) {public static Shooter2 m_shooter = null;
        // set the default command for a subsystem here.
        //setDefualtCommand(new MySpecialCommand());
       // {
    //Robotmap1.Java () {
        //Solenoids
       // public static final int SHOOTER_PITCH_SOLENOID_DEPLOY = 0;
       // public static final int SHOOTER_PITCH_SOLENOID_RETRACT = 1;
    

//{ 

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;

  // Create a new RobotMap to have access to the different parts of the robot.
  Systems system = new Systems();
  Drivetrain drivetrain = new Drivetrain(this);
  Input input = system.new Input();
  //public static SubLift sublift = new SubLift();
  ControlPanel controlPanel = system.new ControlPanel();
  Shooter shooter = system.new Shooter();
  public static IntakeSubsystem intake = new IntakeSubsystem(); // system.new IntakeSubsystem();
 // ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    

 // ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
   //Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
  
  // public final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.OUTPUT, ChannelMode.OUTPUT);
  

  

  Timer timer = new Timer();  
  double lastCompletedTime = 0.0;
  int currentStep = 0;
  double[] autonTimes = new double[2];

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Robot() {
    // Configure the button bindings
    // autoCommand = new DriveTime(.5, 2, drivetrain);
    autoCommand = new DriveTurnDrive(drivetrain);
    configureButtonBindings();
  }

  //public static int getKsolenoidbutton() {
    //return kSolenoidButton;
  //}

  //public DoubleSolenoid getM_doubleSolenoid() {
    //return m_doubleSolenoid;
  //}

  public Solenoid getM_solenoid() {
    return m_solenoid;
  }

  public static int getKdoublesolenoidreverse() {
    return kDoubleSolenoidReverse;
  }

  public static int getKdoublesolenoidforward() {
    return kDoubleSolenoidForward;
  }

  public static int getKsolenoidbutton() {
    return kSolenoidButton;
  }

  public DoubleSolenoid getM_doubleSolenoid() {
    return m_doubleSolenoid;
  }

  public Joystick getLeft() {
    return left;
  }

  //public Solenoid getM_solenoid() {
    //return m_solenoid;
  //}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_chooser.setDefaultOption("Auto Routine Distance", autoCommand);
    SmartDashboard.putData(m_chooser);
  }
  private final Joystick left = new Joystick(0);
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM,0);
  private final DoubleSolenoid m_doubleSolenoid =
     new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
 
  private static final int kSolenoidButton = 4;
 private static final int kDoubleSolenoidForward = 6;
 private static final int kDoubleSolenoidReverse = 4;

 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
    public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain.setTargetHeadingToCurrentHeading();
  
    CameraServer.startAutomaticCapture("frontCam", 0);
    CameraServer.startAutomaticCapture("backCam", 1);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // System.out.println("new code gyro angle: " + drivetrain.gyro.getAngle());
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    drivetrain.setTargetHeadingToCurrentHeading();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    drivetrain.setTargetHeadingToCurrentHeading();
    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drivetrain.setTargetHeadingToCurrentHeading();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.println("Gyro Angle:" + (int) drivetrain.gyro.getAngleZ());
    if (!input.isDrivetrainLocked()) {
      if (Constants.ARCADE_DRIVE = true) {
        drivetrain.moveArcadeLinear(input.getLeftDrive(), input.getRightDrive() * -1);
      }
      else {
       drivetrain.moveLinear(input.getLeftDrive(), input.getRightDrive());
      }
       } else {
      drivetrain.moveLinear(0,0);
    }


    if (input.getControlPanelActive()) controlPanel.Move(0.5f);
    else controlPanel.Stop();
    if (input.getShoot()) { 
      shooter.shoot(input.shooterIntensity());
    }
    else {
      shooter.stopShoot();
    } 

    if (input.getRaise()) { 
      shooter.raise(); 
      shooter.pickUp();
    } 
    else if (!input.getRaise() && !input.getShoot() && !input.getLower()) {
      shooter.stopRaise();
      shooter.stopPickUp();
    }

    if (input.getLower()) {
      shooter.drop();
    }

    if (input.liftBot()) {
      intake.LiftBot();
    } else if (input.lowerBot()) {
      intake.LowerBot();
    } else {
      intake.StopLift();
    }

    if (input.liftClaw()) {
      intake.outtake();
    } else if (input.lowerClaw()) {
      intake.intake();
    } else {
      intake.stop();
    }
   m_solenoid.set(left.getRawButton(kSolenoidButton));
  if (left.getRawButton(kDoubleSolenoidForward)) {
   m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
   } else if (left.getRawButton(kDoubleSolenoidReverse)) {
   m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
