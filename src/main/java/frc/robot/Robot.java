// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Imports that allow the usage of REV Spark Max motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kNothingAuto = "do nothing";
  private static final String kLaunchAndDrive = "launch drive";
  private static final String kLaunch = "launch";
  private static final String kDrive = "drive";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static final int kLeftRearID = 11;
  public static final int kLeftFrontID = 10;
  public static final int kRightRearID = 13;
  public static final int kRightFrontID = 12;

  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, rTolerance;


  // defines motors
  CANSparkMax leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushed);
  CANSparkMax leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushed);
  CANSparkMax rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushed);
  CANSparkMax rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushed);
  
  private SparkPIDController rightPID;


  // encoder vars

  RelativeEncoder rightEncoder = rightFront.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8092);
  RelativeEncoder leftEncoder = leftFront.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8092);


  DifferentialDrive m_drivetrain;

  Joystick m_driverController = new Joystick(0);
  Joystick m_manipController = new Joystick(1);

  // --------------- Magic numbers. Use these to adjust settings. ---------------

  /**
   *  * How many amps can an individual drivetrain motor use.
   *  
   */
  static final int DRIVE_CURRENT_LIMIT_A = 60;

  /**
   *  * How many amps the feeder motor can use.
   *  
   */
  static final int FEEDER_CURRENT_LIMIT_A = 60;

  /**
   *  * Percent output to run the feeder when expelling note
   *  
   */
  static final double FEEDER_OUT_SPEED = 1.0;

  /**
   *  * Percent output to run the feeder when intaking note
   *  
   */
  static final double FEEDER_IN_SPEED = -.4;

  /**
   *  * Percent output for amp or drop note, configure based on polycarb bend
   *  
   */
  static final double FEEDER_AMP_SPEED = .4;

  /**
   *  * How many amps the launcher motor can use.
   *  *
   *  * In our testing we favored the CIM over NEO, if using a NEO lower this to
   * 60
   *  
   */
  static final int LAUNCHER_CURRENT_LIMIT_A = 60;

  /**
   *  * Percent output to run the launcher when intaking AND expelling note
   *  
   */
  static final double LAUNCHER_SPEED = 1.0;

  /**
   *  * Percent output for scoring in amp or dropping note, configure based on
   * polycarb bend
   * .14 works well with no bend from our testing
   *  
   */
  static final double LAUNCHER_AMP_SPEED = .17;
  /**
   * Percent output for the roller claw
   */
  static final double CLAW_OUTPUT_POWER = .5;
  /**
   * Percent output to help retain notes in the claw
   */
  static final double CLAW_STALL_POWER = .1;
  /**
   * Percent output to power the climber
   */
  static final double CLIMER_OUTPUT_POWER = 1;

  // public double kP, kI, kD, iLimit;
  static final double ticksToFeet = ((1 / 8196) * (846 / 100) * 6 * Math.PI);


  // double kP = 0.05;
  // double kI = 0.05;
  // double kD = 0.1;
// PIDController pidController = new PIDController(kP, kI, kD);


  // public void moveToPosPID(double targpos) {
  //   .setSetpoint(targpos);


  //   while (Math.abs(rightEncoder.getPosition()) < targpos) {
  //     double outputPowerRight = pidController.calculate(rightEncoder.getPosition()); // encoder gets the rpm of the encoder, then pid calculates the rpm that it needs to maintain to get to the target position
  //     double outputPowerLeft = pidController.calculate(leftEncoder.getPosition());

  //     SmartDashboard.putNumber("EncoderRight", rightEncoder.getPosition());
  //     SmartDashboard.putNumber("EncoderLeft", leftEncoder.getPosition());

  //     rightFront.set(outputPowerRight);
  //     leftFront.set(outputPowerLeft);
  //   }
   
  //   rightFront.stopMotor();
  //   leftFront.stopMotor();
  // }

  @Override
  public void robotInit() {
    rightPID = rightFront.getPIDController();
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("launch note and drive", kLaunchAndDrive);
    m_chooser.addOption("launch", kLaunch);
    m_chooser.addOption("drive", kDrive);
    SmartDashboard.putData("Auto choices", m_chooser);

    kP = 1;
    kI = 1;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;
    rTolerance = .02;

    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
    rightPID.setIZone(kIz);
    rightPID.setFF(kFF);
    rightPID.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("right P", kP);
    SmartDashboard.putNumber("right I", kI);
    SmartDashboard.putNumber("right D", kD);
    SmartDashboard.putNumber("right Iz", kIz);
    SmartDashboard.putNumber("right FF", kFF);
    SmartDashboard.putNumber("right Max Output", kMaxOutput);
    SmartDashboard.putNumber("right Min Output", kMinOutput);
    SmartDashboard.putNumber("right Set Rotations", 100);

    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("EncoderRight", rightEncoder.getPosition());
    SmartDashboard.putNumber("EncoderLeft", leftEncoder.getPosition());



    /*
     *  * Apply the current limit to the drivetrain motors
     *  
     */
    leftRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    leftFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    rightRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    rightFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftFront.setInverted(true);
    rightFront.setInverted(false);

    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  public void EncoderGoToPos(int TargPos) {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    double curposRight = rightEncoder.getPosition();
    double curposLeft = leftEncoder.getPosition();
    m_drivetrain.arcadeDrive(0.5, 0);
    while (curposRight < TargPos && curposLeft < TargPos) {
    SmartDashboard.putNumber("EncoderRight", rightEncoder.getPosition());
    SmartDashboard.putNumber("EncoderLeft", leftEncoder.getPosition());
      curposRight = rightEncoder.getPosition();
      curposLeft = leftEncoder.getPosition();
      
    }
  }

  @Override
  public void robotPeriodic() {
    final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
    final CommandXboxController m_operatorController = new CommandXboxController(kOperatorControllerPort);
    m_drivetrain.arcadeDrive(0.5 * -m_driverController.getLeftY(), 0.5 * -m_driverController.getRightX());

    // SmartDashboard
    SmartDashboard.putNumber("Left Y", m_driverController.getLeftY());
    SmartDashboard.putNumber("Left X", m_driverController.getRightX());
    SmartDashboard.putNumber("EncoderRight", rightEncoder.getPosition());
    SmartDashboard.putNumber("EncoderLeft", leftEncoder.getPosition());


  }

  /*
   *  * Auto constants, change values below in autonomousInit()for different
   * autonomous behaviour
   *  *
   *  * A delayed action starts X seconds into the autonomous period
   *  *
   *  * A time action will perform an action for X amount of seconds
   *  *
   *  * Speeds can be changed as desired and will be set to 0 when
   *  * performing an auto that does not require the system
   *  
   */
  double AUTO_LAUNCH_DELAY_S;
  double AUTO_DRIVE_DELAY_S;

  double AUTO_DRIVE_TIME_S;

  double AUTO_DRIVE_SPEED;
  double AUTO_LAUNCHER_SPEED;

  double autonomousStartTime;

  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();

    // rightEncoder.setPosition(0);
    // leftEncoder.setPosition(0);

    // AUTO_LAUNCH_DELAY_S = 2;
    // AUTO_DRIVE_DELAY_S = 3;

    // AUTO_DRIVE_TIME_S = 2.0;
    // AUTO_DRIVE_SPEED = -0.5;
    // AUTO_LAUNCHER_SPEED = 1;

    // leftRear.set(1);

    /*
     *  * Depeding on which auton is selected, speeds for the unwanted subsystems
     * are set to 0
     *  * if they are not used for the selected auton
     *  *
     *  * For kDrive you can also change the kAutoDriveBackDelay
     *  
     */
    // if (m_autoSelected == kLaunch) {
    //   AUTO_DRIVE_SPEED = 0;
    // } else if (m_autoSelected == kDrive) {
    //   AUTO_LAUNCHER_SPEED = 0;
    // } else if (m_autoSelected == kNothingAuto) {
    //   AUTO_DRIVE_SPEED = 0;
    //   AUTO_LAUNCHER_SPEED = 0;
    // }

    // autonomousStartTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  // @Override
  public void autonomousPeriodic() {
    // EncoderGoToPos(50);
    // moveToPosPID(10);
    
    double p = SmartDashboard.getNumber("P Gain", kP);
    double i = SmartDashboard.getNumber("I Gain", kI);
    double d= SmartDashboard.getNumber("D Gain", kD);
    double iz = SmartDashboard.getNumber("I zone", kIz);
    double ff = SmartDashboard.getNumber("Feed Forward", kFF);
    double max = SmartDashboard.getNumber("Max Output", kMaxOutput);
    double min = SmartDashboard.getNumber("Min Output", kMinOutput);
    double rotations = SmartDashboard.getNumber("Set Rotations", 100);
  
    
    //if pid changed on smart dash board
    if((p != kP)) {rightPID.setP(p); kP = p;};
    if((i != kI)) {rightPID.setI(p); kI = i;};
    if((d != kD)) {rightPID.setD(p); kD = d;};
    if((iz != kIz)) {rightPID.setIZone(iz); kIz = iz;};
    if((ff) != kFF) {rightPID.setFF(ff); kFF = ff;};
    if((max != kMaxOutput) || (min != kMinOutput)) {
      rightPID.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
    };
    SmartDashboard.putNumber("SetPoint", rotations);
    rightPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  // double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

  // /*
  //  * Spins up launcher wheel until time spent in auto is greater than
  // AUTO_LAUNCH_DELAY_S
  //  *
  //  * Feeds note to launcher until time is greater than AUTO_DRIVE_DELAY_S
  //  *
  //  * Drives until time is greater than AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S
  //  *
  //  * Does not move when time is greater than AUTO_DRIVE_DELAY_S +
  // AUTO_DRIVE_TIME_S
  //  */
  // if(timeElapsed < AUTO_LAUNCH_DELAY_S)
  // {
  // m_launchWheel.set(AUTO_LAUNCHER_SPEED);
  // m_drivetrain.arcadeDrive(0, 0);

  // }
  // else if(timeElapsed < AUTO_DRIVE_DELAY_S)
  // {
  // m_feedWheel.set(AUTO_LAUNCHER_SPEED);
  // m_drivetrain.arcadeDrive(0, 0);
  // }
  // else if(timeElapsed < AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S)
  // {
  // m_launchWheel.set(0);
  // m_feedWheel.set(0);
  // m_drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, 0);
  // }
  // else
  // {
  // m_drivetrain.arcadeDrive(0, 0);
  // }
  // /* For an explanation on differintial drive, squaredInputs, arcade drive and
  // tank drive see the bottom of this file */
  // }

  // /** This function is called once when teleop is enabled. */
  // @Override
  // public void teleopInit() {
  // /*
  //  * Motors can be set to idle in brake or coast mode.
  //  *
  //  * Brake mode effectively shorts the leads of the motor when not running,
  // making it more
  //  * difficult to turn when not running.
  //  *
  //  * Coast doesn't apply any brake and allows the motor to spin down naturally
  // with the robot's momentum.
  //  *
  //  * (touch the leads of a motor together and then spin the shaft with your
  // fingers to feel the difference)
  //  *
  //  * This setting is driver preference. Try setting the idle modes below to
  // kBrake to see the difference.
  //  */
  // leftRear.setIdleMode(IdleMode.kCoast);
  // leftFront.setIdleMode(IdleMode.kCoast);
  // rightRear.setIdleMode(IdleMode.kCoast);
  // rightFront.setIdleMode(IdleMode.kCoast);
  // }

  // /** This function is called periodically during operator control. */
  // @Override
  // public void teleopPeriodic() {

  // /*
  //  * Spins up the launcher wheel
  //  */
  // if (m_manipController.getRawButton(1)) {
  // m_launchWheel.set(LAUNCHER_SPEED);
  // }
  // else if(m_manipController.getRawButtonReleased(1))
  // {
  // m_launchWheel.set(0);
  // }

  // /*
  //  * Spins feeder wheel, wait for launch wheel to spin up to full speed for
  // best results
  //  */
  // if (m_manipController.getRawButton(6))
  // {
  // m_feedWheel.set(FEEDER_OUT_SPEED);
  // }
  // else if(m_manipController.getRawButtonReleased(6))
  // {
  // m_feedWheel.set(0);
  // }

  // /*
  //  * While the button is being held spin both motors to intake note
  //  */
  // if(m_manipController.getRawButton(5))
  // {
  // m_launchWheel.set(-LAUNCHER_SPEED);
  // m_feedWheel.set(FEEDER_IN_SPEED);
  // }
  // else if(m_manipController.getRawButtonReleased(5))
  // {
  // m_launchWheel.set(0);
  // m_feedWheel.set(0);
  // }

  // /*
  //  * While the amp button is being held, spin both motors to "spit" the note
  //  * out at a lower speed into the amp
  //  *
  //  * (this may take some driver practice to get working reliably)
  //  */
  // if(m_manipController.getRawButton(2))
  // {
  // m_feedWheel.set(FEEDER_AMP_SPEED);
  // m_launchWheel.set(LAUNCHER_AMP_SPEED);
  // }
  // else if(m_manipController.getRawButtonReleased(2))
  // {
  // m_feedWheel.set(0);
  // m_launchWheel.set(0);
  // }

  // /**
  // * Hold one of the two buttons to either intake or exjest note from roller
  // claw
  // *
  // * One button is positive claw power and the other is negative
  // *
  // * It may be best to have the roller claw passively on throughout the match to
  // * better retain notes but we did not test this
  // */
  // if(m_manipController.getRawButton(3))
  // {
  // m_rollerClaw.set(CLAW_OUTPUT_POWER);
  // }
  // else if(m_manipController.getRawButton(4))
  // {
  // m_rollerClaw.set(-CLAW_OUTPUT_POWER);
  // }
  // else
  // {
  // m_rollerClaw.set(0);
  // }

  // /**
  // * POV is the D-PAD (directional pad) on your controller, 0 == UP and 180 ==
  // DOWN
  // *
  // * After a match re-enable your robot and unspool the climb
  // */
  // // if(m_manipController.getPOV() == 0)
  // // {
  // // m_climber.set(1);
  // // }
  // // else if(m_manipController.getPOV() == 180)
  // // {
  // // m_climber.set(-1);
  // // }
  // // else
  // // {
  // // m_climber.set(0);
  // // }

  // /*
  //  * Negative signs are here because the values from the analog sticks are
  // backwards
  //  * from what we want. Pushing the stick forward returns a negative when we
  // want a
  //  * positive value sent to the wheels.
  //  *
  //  * If you want to change the joystick axis used, open the driver station, go
  // to the
  //  * USB tab, and push the sticks determine their axis numbers
  // *
  // * This was setup with a logitech controller, note there is a switch on the
  // back of the
  // * controller that changes how it functions
  //  */
  // m_drivetrain.arcadeDrive(-m_driverController.getRawAxis(1),
  // -m_driverController.getRawAxis(4), false);
  // }
}

/*
 *  * The kit of parts drivetrain is known as differential drive, tank drive or
 * skid-steer drive.
 *  *
 *  * There are two common ways to control this drivetrain: Arcade and Tank
 *  *
 *  * Arcade allows one stick to be pressed forward/backwards to power both
 * sides of the drivetrain to move straight forwards/backwards.
 *  * A second stick (or the second axis of the same stick) can be pushed
 * left/right to turn the robot in place.
 *  * When one stick is pushed forward and the other is pushed to the side, the
 * robot will power the drivetrain
 *  * such that it both moves fowards and turns, turning in an arch.
 *  *
 *  * Tank drive allows a single stick to control of a single side of the robot.
 *  * Push the left stick forward to power the left side of the drive train,
 * causing the robot to spin around to the right.
 *  * Push the right stick to power the motors on the right side.
 *  * Push both at equal distances to drive forwards/backwards and use at
 * different speeds to turn in different arcs.
 *  * Push both sticks in opposite directions to spin in place.
 *  *
 *  * arcardeDrive can be replaced with tankDrive like so:
 *  *
 *  * m_drivetrain.tankDrive(-m_driverController.getRawAxis(1),
 * -m_driverController.getRawAxis(5))
 *  *
 *  * Inputs can be squared which decreases the sensitivity of small drive
 * inputs.
 *  *
 *  * It literally just takes (your inputs * your inputs), so a 50% (0.5) input
 * from the controller becomes (0.5 * 0.5) -> 0.25
 *  *
 *  * This is an option that can be passed into arcade or tank drive:
 *  * arcadeDrive(double xSpeed, double zRotation, boolean squareInputs)
 *  *
 *  *
 *  * For more information see:
 *  * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-
 * drive-classes.html
 *  *
 *  *
 * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/
 * wpi/first/wpilibj/drive/DifferentialDrive.java
 *  *
 *  
 */