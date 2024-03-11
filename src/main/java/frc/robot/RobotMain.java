// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.math.BigInteger;

// Imports that allow the usage of REV Spark Max motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.*;//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

public class RobotMain extends TimedRobot {
 
  // Smart Dashboard
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String Redrightauto = "Red right";
  private static final String Redmiddleauto = "Red middle";
  private static final String Redleftauto = "Red left";
  private static final String Bluerightauto = "Blue right";
  private static final String Bluemiddleauto = "Blue middle";
  private static final String Blueleftauto = "Blue left";
  private String chosenauto;


  // editable variables
  public static final int kLeftRearID = 11;
  public static final int kLeftFrontID = 10;
  public static final int kRightRearID = 13;
  public static final int kRightFrontID = 12;

  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  


  // variables here
  public double turbo_multi=.25;

  // define values for position PID
  public double rightkP_P = 0.1;
  public double rightkI_P = 0;
  public double rightkD_P = 0;
  public double rightkIz_P = 0;
  public double rightkFF_P = 0;
  public double rightkMax_P = -0.4;
  public double rightkMin_P = 0.4;
  public double leftkP_P = 0.1;
  public double leftkI_P = 0;
  public double leftkD_P = 0;
  public double leftkIz_P = 0;
  public double leftkFF_P = 0;
  public double leftkMax_P = -0.4;
  public double leftkMin_P = 0.4;
  

  // define separate values for velocity PID
  public double rightkP_V = 0.0001;
  public double rightkI_V = 0;
  public double rightkD_V = 0;
  public double rightkIz_V = 0;
  public double rightkFF_V = 0;
  public double rightkMax_V = -0.4;
  public double rightkMin_V = 0.4;
  public double leftkP_V = 0.0001;
  public double leftkI_V = 0;
  public double leftkD_V = 0;
  public double leftkIz_V = 0;
  public double leftkFF_V = 0;
  public double leftkMax_V = -0.4;
  public double leftkMin_V = 0.4;
  
  // define drivetrain motors
  private CANSparkMax leftRearDrivetrainMotor;
  private CANSparkMax leftFrontDrivetrainMotor;
  private CANSparkMax rightRearDrivetrainMotor;
  private CANSparkMax rightFrontDrivetrainMotor;

  // define arm motors
  private CANSparkMax rightArmMotor;
  private CANSparkMax leftArmMotor;

  // define intake motors
  private CANSparkMax intakeMotor;
  private CANSparkMax lowerLaunchMotor;
  private CANSparkMax upperLaunchMotor;
  
  // define pid controllers
  private SparkPIDController rightDrivetrainPID;
  private SparkPIDController leftDrivetrainPID;
  private SparkPIDController rightArmMotorPID;
  private SparkPIDController leftArmMotorPID;

  // define drivetrain encoders
  private RelativeEncoder rightDrivetrainEncoder;
  private RelativeEncoder leftDrivetrainEncoder;
  private RelativeEncoder rightArmMotorEncoder;
  private RelativeEncoder leftArmMotorEncoder;
  private RelativeEncoder lowerLaunchMotorEncoder;
  private RelativeEncoder upperLaunchMotorEncoder;


  // create differential drive (tank)
  DifferentialDrive drivetrain;

  // define controllers
  Joystick driverController = new Joystick(kDriverControllerPort);
  Joystick operatorController = new Joystick(kOperatorControllerPort);

  public static final ADIS16470_IMU gyro = new ADIS16470_IMU();
  
  // editable variables
  static final int drivetrainCurrentLimit = 60;
  static final int intakeCurrentLimit = 10;
  static final int armCurrentLimit = 30;
  static final int flywheelCurrentLimit = 10;
  static final int liftCurrentLimit = 30;

  static final double drivetrainRightModifier = 0.5;
  static final double drivetrainLeftModifier = 0.5;

  static final double intakeInSpeed = 0.3;
  static final double intakeOutSpeed = 0.3;
  static final double flywheelSpeakerSpeed = .7;
  static final double flywheelAmpSpeed = .4;
  static final double flywheelTrapSpeed = .3;
  static final double liftTensionSpeed = .3;
  static final double liftReleaseSpeed = .1;

  static final double ticksToFeet = ((1 / 8196) * (846 / 100) * 6 * Math.PI);

  // pid error vars

  public double errorSum = 0;
  public double changablePosition = 10;
  public double lastTimeStamp = 0;
  public double lastError = 0;

  public double rightDriveDistance = 0;
  public double leftDriveDistance = 0;
  public double rightArmPosit=0;
  public double leftArmPosit=0;
  public double armTarget=0;
  public boolean armJoystick = true;


public void setIntakeMotor(double percent, int amps) {
  intakeMotor.set(percent);
  intakeMotor.setSmartCurrentLimit(amps);
  SmartDashboard.putNumber("intake power (%)", percent);
  SmartDashboard.putNumber("intake motor current (amps)", intakeMotor.getOutputCurrent());
  SmartDashboard.putNumber("intake motor temperature (C)", intakeMotor.getMotorTemperature());
}
public void setLaunchMotor(double percent, int amps) {
  upperLaunchMotor.set(percent);
  lowerLaunchMotor.set(percent);
  upperLaunchMotor.setSmartCurrentLimit(amps);
  lowerLaunchMotor.setSmartCurrentLimit(amps);
  SmartDashboard.putNumber("Launch power (%)", percent);
  SmartDashboard.putNumber("Upper Launch motor current (amps)", upperLaunchMotor.getOutputCurrent());
  SmartDashboard.putNumber("Upper Launch motor temperature (C)", upperLaunchMotor.getMotorTemperature());
  SmartDashboard.putNumber("Lower Launch motor current (amps)", lowerLaunchMotor.getOutputCurrent());
  SmartDashboard.putNumber("Lower Launch motor temperature (C)", lowerLaunchMotor.getMotorTemperature());
}
public void setArmMotor(double percent, int amps) {
  rightArmPosit  = rightArmMotorEncoder.getPosition();
  leftArmPosit = leftArmMotorEncoder.getPosition();
  if (rightArmPosit>=45.5 | leftArmPosit<=-45.5) {percent=.5*percent;}
  if (rightArmPosit<=-6 | leftArmPosit>=6) {percent=.5*percent;}
  if (rightArmPosit>=46.8 | leftArmPosit<=-46.8) {percent=-.0251;}
  if (rightArmPosit<=-8 | leftArmPosit>=8) {percent=.02751;}

  rightArmMotor.set(percent);
  leftArmMotor.set(-percent);
  rightArmMotor.setSmartCurrentLimit(amps);
  leftArmMotor.setSmartCurrentLimit(amps);
  SmartDashboard.putNumber("Arm power (%)", percent);
  SmartDashboard.putNumber("Right Arm current (amps)", rightArmMotor.getOutputCurrent());
  SmartDashboard.putNumber("Right Arm motor temperature (c)", rightArmMotor.getMotorTemperature());
  SmartDashboard.putNumber("Left Arm current (amps)", leftArmMotor.getOutputCurrent());
  SmartDashboard.putNumber("Left Arm motor temperature (c)", leftArmMotor.getMotorTemperature());
}
  @Override
  public void robotInit() {
    
  leftRearDrivetrainMotor = new CANSparkMax(kLeftRearID, MotorType.kBrushed);
  leftFrontDrivetrainMotor = new CANSparkMax(kLeftFrontID, MotorType.kBrushed);
  rightRearDrivetrainMotor = new CANSparkMax(kRightRearID, MotorType.kBrushed);
  rightFrontDrivetrainMotor = new CANSparkMax(kRightFrontID, MotorType.kBrushed);

  leftArmMotor = new CANSparkMax(17, MotorType.kBrushless);
  rightArmMotor = new CANSparkMax(18,MotorType.kBrushless);

  lowerLaunchMotor = new CANSparkMax(16,MotorType.kBrushless);
  upperLaunchMotor = new CANSparkMax(15, MotorType.kBrushless);

  intakeMotor = new CANSparkMax(19, MotorType.kBrushless);

  rightDrivetrainEncoder = rightFrontDrivetrainMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8192);
  leftDrivetrainEncoder = leftFrontDrivetrainMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8192);
  
  lowerLaunchMotorEncoder = lowerLaunchMotor.getEncoder();
  upperLaunchMotorEncoder = upperLaunchMotor.getEncoder();

  leftArmMotorEncoder = leftArmMotor.getEncoder();
  rightArmMotorEncoder = rightArmMotor.getEncoder();

  leftArmMotorEncoder.setPosition(0);
  rightArmMotorEncoder.setPosition(0);
//remove arm inversion - used negative value to joystick. Inversion messes with PID

  rightArmMotorPID = rightArmMotor.getPIDController();
  leftArmMotorPID = leftArmMotor.getPIDController();

//remove encoder set position. Will try other locations...
 
  leftFrontDrivetrainMotor.setInverted(true);
  rightFrontDrivetrainMotor.setInverted(true);

  leftRearDrivetrainMotor.follow(leftFrontDrivetrainMotor);
  rightRearDrivetrainMotor.follow(rightFrontDrivetrainMotor);
    
  rightDrivetrainPID = rightFrontDrivetrainMotor.getPIDController();
  leftDrivetrainPID = leftFrontDrivetrainMotor.getPIDController();

  CameraServer.startAutomaticCapture();//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  
    kP = 1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    rightDrivetrainPID.setP(kP);
    rightDrivetrainPID.setI(kI);
    rightDrivetrainPID.setD(kD);
    rightDrivetrainPID.setIZone(kIz);
    rightDrivetrainPID.setFF(kFF);
    rightDrivetrainPID.setOutputRange(kMinOutput, kMaxOutput);

    leftDrivetrainPID.setP(kP);
    leftDrivetrainPID.setI(kI);
    leftDrivetrainPID.setD(kD);
    leftDrivetrainPID.setIZone(kIz);
    leftDrivetrainPID.setFF(kFF);
    leftDrivetrainPID.setOutputRange(kMinOutput, kMaxOutput);

    rightArmMotorPID.setP(.05);
    rightArmMotorPID.setI(0);
    rightArmMotorPID.setD(0);
    rightArmMotorPID.setIZone(0);
    rightArmMotorPID.setFF(0);
    rightArmMotorPID.setOutputRange(-.3, .3);

    leftArmMotorPID.setP(.05);
    leftArmMotorPID.setI(0);
    leftArmMotorPID.setD(0);
    leftArmMotorPID.setIZone(0);
    leftArmMotorPID.setFF(0);
    leftArmMotorPID.setOutputRange(-.3,.3);

    leftArmMotor.setOpenLoopRampRate(.33);
    rightArmMotor.setOpenLoopRampRate(.33);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    // add auton choices
    autoChooser.setDefaultOption("Red right", Redrightauto);
    autoChooser.addOption("Red middle", Redmiddleauto);
    autoChooser.addOption("Red left", Redleftauto);
    autoChooser.addOption("Blue right", Bluerightauto);
    autoChooser.addOption("Blue middle", Bluemiddleauto);
    autoChooser.addOption("Blue left", Blueleftauto);
    SmartDashboard.putData("Autonomus Choices", autoChooser);

    // Apply the current limit to the drivetrain motors
    leftRearDrivetrainMotor.setSmartCurrentLimit(drivetrainCurrentLimit);
    leftFrontDrivetrainMotor.setSmartCurrentLimit(drivetrainCurrentLimit);
    rightRearDrivetrainMotor.setSmartCurrentLimit(drivetrainCurrentLimit);
    rightFrontDrivetrainMotor.setSmartCurrentLimit(drivetrainCurrentLimit);

    drivetrain = new DifferentialDrive(leftFrontDrivetrainMotor, rightFrontDrivetrainMotor);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Right Arm Encoder Value", rightArmMotorEncoder.getPosition());
    SmartDashboard.putNumber("Left Arm Encoder Value", leftArmMotorEncoder.getPosition());
    SmartDashboard.putNumber("Lower Launcher Velocity", lowerLaunchMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Upper Launcher Velocity", upperLaunchMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Right ProcessVariable", rightDrivetrainEncoder.getPosition());
    SmartDashboard.putNumber("Left ProcessVariable", leftDrivetrainEncoder.getPosition());
    // removed println command with weird values???
  }
  @Override
  public void teleopInit() {
      rightFrontDrivetrainMotor.setInverted(false); 

  }
  
  @Override
  public void teleopPeriodic() {
    armTarget= rightArmMotorEncoder.getPosition();
  //remove xboxcontroller definitions. Used Joystick, definition earlier in code
   
   
  if (driverController.getRawButtonPressed(1))
    {
      // X
      turbo_multi = 0.5;
    } else if (driverController.getRawButtonPressed(2))
    {
      // A
      turbo_multi = 0.33;
    } else if (driverController.getRawButtonPressed(3))
    {
      // B
      turbo_multi = 1.0;
    } else if (driverController.getRawButtonPressed(4))
    {
      // Y
      turbo_multi = 0.75;
    }


  drivetrain.arcadeDrive(turbo_multi * -driverController.getRawAxis(1), turbo_multi * -driverController.getRawAxis(4),true);

  if (operatorController.getRawButtonPressed(1)) {
      setIntakeMotor(.5, 20);
    }
  if (operatorController.getRawButtonReleased(1)) {
      setIntakeMotor(0, 20);
    }
  if (operatorController.getRawButtonPressed(4)) {
      setIntakeMotor(-.5, 25);
    }
  if (operatorController.getRawButtonReleased(4)) {
      setIntakeMotor(0, 25);
    }
  if (operatorController.getRawButton(2)) {
      setLaunchMotor(.7, 25);
    }
  if (operatorController.getRawButton(3)) {
      setLaunchMotor(0, 25);
    }
  if (operatorController.getRawButton(5)) { 
      armTarget=45.5;
    }
  if (operatorController.getRawButton(6)) { 
      armTarget=35;
       }
  if (operatorController.getRawButton(7)) { 
      armTarget=-10;
       
    }
    if (operatorController.getRawAxis(2)>0.7) { 
        armJoystick=false; 
    }
    
    if (operatorController.getRawAxis(3)>0.7 ){
        armJoystick = true;
    }
    
    double armAxis = Math.pow(operatorController.getRawAxis(1),3)*.3;
  
    if (armJoystick==true) {
    setArmMotor(armAxis, 25);
    }
    else{
      if (rightArmMotorEncoder.getPosition() - armTarget >=.5 || rightArmMotorEncoder.getPosition() - armTarget <=-.5 ) {armPidcontrol(armTarget);}
      if (rightArmMotorEncoder.getPosition()<=-8) {armPidcontrol(-7);}

    }
    SmartDashboard.putNumber("armTarget", armTarget);

    
SmartDashboard.putBoolean("Arm Joystick", armJoystick);

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
 
  
  public void armPidcontrol(double target) {
      rightArmMotorPID.setReference(target, CANSparkMax.ControlType.kPosition);
      leftArmMotorPID.setReference(-target, CANSparkMax.ControlType.kPosition);
  }

  public void PIDDriveTurn(double angle) {
    gyro.reset();
    errorSum = 0;
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = 0;
    double firstpos = gyro.getAngle();
    while (angle + firstpos > gyro.getAngle()) {
      double currentangle = gyro.getAngle();

      double error = angle - currentangle;
      double dt = Timer.getFPGATimestamp() - lastTimeStamp;

      // accounts for friction is the error is below 1
      if (Math.abs(error) < 1) {
        errorSum += error * dt;
      }

      double errorRate = (error - lastError) / dt;

      double speedRobotRight = (rightkP_P * error) + (rightkI_P * errorSum) + (rightkD_P * errorRate);
      double speedRobotLeft = (leftkP_P * error) + (leftkI_P * errorSum) + (leftkD_P * errorRate);

      drivetrain.tankDrive(speedRobotLeft, speedRobotRight);

      lastTimeStamp = Timer.getFPGATimestamp();
      lastError = error;
      SmartDashboard.putNumber("Gyro while Auton", gyro.getAngle());
    }
  }
  double wheelcircumferencetofeet = (6 * Math.PI) / 12;

  public void PIDDriveFeet(double feet) {
    rightDrivetrainPID.setReference(feet / wheelcircumferencetofeet, CANSparkMax.ControlType.kPosition);
    leftDrivetrainPID.setReference(feet / wheelcircumferencetofeet, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void autonomousInit() {
    autonomousStartTime=Timer.getFPGATimestamp();
    PIDDriveFeet(0);
    chosenauto = autoChooser.getSelected();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    drivetrain.feed();

    /* Comment out code for tuning PID - leaving it for future reference / use if need to reactivate
     double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
  

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { m_pidController12.setP(p); kP = p; }
    if((p != kP)) { rightDrivetrainPID.setP(p); kP = p; leftDrivetrainPID.setP(p); }
    if((i != kI)) { rightDrivetrainPID.setI(i); kI = i; leftDrivetrainPID.setI(i); }
    if((d != kD)) { rightDrivetrainPID.setD(d); kD = d; leftDrivetrainPID.setD(d);}
    if((iz != kIz)) { rightDrivetrainPID.setIZone(iz); kIz = iz; leftDrivetrainPID.setIZone(iz); }
    if((ff != kFF)) { rightDrivetrainPID.setFF(ff); kFF = ff; leftDrivetrainPID.setFF(ff); }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      rightDrivetrainPID.setOutputRange(max, min);
      leftDrivetrainPID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; } 
    
     */
    
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    SmartDashboard.putNumber("time elapsed", timeElapsed);


    switch (chosenauto) {
      case Redrightauto:
        // go forward, turn around 130 degrees, shoot the note, turn back around, go forward, pick up note, and place it in the amp
        System.out.println("first one is working");
        PIDDriveFeet(2);
        PIDDriveTurn(130);
        PIDDriveFeet(1);
        
        // wait for a few seconds (idk how to wait)

        // pick up note
        break;
      case Redmiddleauto:
        // go forward, turn 150 degrees, shoot note, turn 150 degrees back, pick up note, and shoot it into speaker.
        PIDDriveFeet(8.916);
        break;
      case Redleftauto:
        // place the note down or shoot it, go forward into the middle and fire all the notes back to our base.
        break;
      case Bluerightauto:
        // place the note down or shoot it, go forward into the middle and fire all the notes back to our base.
        break;
      case Bluemiddleauto:
        // go forward, turn 150 degrees, shoot note, turn 150 degrees back, pick up note, and shoot it into speaker.
        break;
      case Blueleftauto:
        // go forward, turn around 130 degrees, shoot the note, turn back around, go forward, pick up note, and place it in the amp
        break;
      default:
        break;
    }

  }
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  
}
