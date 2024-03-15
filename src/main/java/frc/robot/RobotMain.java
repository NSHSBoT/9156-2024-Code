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
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy; 
import edu.wpi.first.cscore.VideoSource;

public class RobotMain extends TimedRobot {
 
  // Smart Dashboard
  //private final SendableChooser<String> autofirstmove = new SendableChooser<>();
  private final SendableChooser<String> autoChooserPosition = new SendableChooser<>();
 // private final SendableChooser<String> autoAction = new SendableChooser<>();
  private static final String Redrightauto = "Red right";
  private static final String Redmiddleauto = "Red middle";
  private static final String Redleftauto = "Red left";
  private static final String Bluerightauto = "Blue right";
  private static final String Bluemiddleauto = "Blue middle";
  private static final String Blueleftauto = "Blue left";
  private static final String onlyShoot = "Only Shoot";
  private static final String ccwExit = "Counter Clockwise Exit";
  private static final String clockwiseExit = "Clockwise Exit";
  private static final String zeroMiddle = "Middle 'ZERO' on ground";

 // private String autofirstchoice;
  private String chosenauto;
 // private String actionauto;


  // editable variables
  public static final int kLeftRearID = 11;
  public static final int kLeftFrontID = 10;
  public static final int kRightRearID = 13;
  public static final int kRightFrontID = 12;

  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  


  // variables here
  public double turbo_multi=.5;

  // define values for position PID
  public double rightkP_P = 0.023;
  public double rightkI_P = 0.075;
  public double rightkD_P = 0.065;
  public double rightkIz_P = 0;
  public double rightkFF_P = 0;
  public double rightkMax_P = 0.75;
  public double rightkMin_P = -0.75;
  public double leftkP_P = 0.023;
  public double leftkI_P = 0.075;
  public double leftkD_P = 0.065;
  public double leftkIz_P = 0;
  public double leftkFF_P = 0;
  public double leftkMax_P = 0.75;
  public double leftkMin_P = -0.75;
  

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

  // define lift motor
  private CANSparkMax liftMotor;

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
  public boolean armJoystick = false;

public void setLiftMotor(double percent, int amps) {
  liftMotor.set(percent);
  liftMotor.setSmartCurrentLimit(amps);
}

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
  if (rightArmPosit>=45.5*1.6 || leftArmPosit<=-45.5*1.6) {percent=.5*percent;}
  if (rightArmPosit<=-6*1.6 || leftArmPosit>=1.6*6) {percent=.5*percent;}
  if (rightArmPosit>=48.5 *1.6|| leftArmPosit<=-48.5*1.6) {percent=-.0251;}
  if (rightArmPosit<=-8*1.6 || leftArmPosit>=8*1.6) {percent=.02751;}

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
  
  gyro.reset();
    
  leftRearDrivetrainMotor = new CANSparkMax(kLeftRearID, MotorType.kBrushed);
  leftFrontDrivetrainMotor = new CANSparkMax(kLeftFrontID, MotorType.kBrushed);
  rightRearDrivetrainMotor = new CANSparkMax(kRightRearID, MotorType.kBrushed);
  rightFrontDrivetrainMotor = new CANSparkMax(kRightFrontID, MotorType.kBrushed);

  leftArmMotor = new CANSparkMax(17, MotorType.kBrushless);
  rightArmMotor = new CANSparkMax(18,MotorType.kBrushless);

  lowerLaunchMotor = new CANSparkMax(16,MotorType.kBrushless);
  upperLaunchMotor = new CANSparkMax(15, MotorType.kBrushless);

  intakeMotor = new CANSparkMax(19, MotorType.kBrushless);

  liftMotor = new CANSparkMax(14,MotorType.kBrushed);

  rightDrivetrainEncoder = rightFrontDrivetrainMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8192);
  leftDrivetrainEncoder = leftFrontDrivetrainMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8192);
  //rightDrivetrainEncoder.setPosition(0);
  //leftDrivetrainEncoder.setPosition(0);

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

   UsbCamera camera1 = CameraServer.startAutomaticCapture(0);
   UsbCamera camera2 = CameraServer.startAutomaticCapture(1);

   camera1.setVideoMode(PixelFormat.kMJPEG, 160,120,10);
   camera2.setVideoMode(PixelFormat.kYUYV, 160,120,10);

   camera1.setExposureAuto();
   camera1.setWhiteBalanceAuto();
   camera2.setExposureAuto();
   camera2.setWhiteBalanceAuto();
   

  
    kP = .52; 
    kI = 0.001;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = .3; 
    kMinOutput = -.3;

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
/* 
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
*/


    // First move
   // autofirstmove.setDefaultOption("Shoot", "Shoot");
   // autofirstmove.addOption("Pause", "Pause");
   // autofirstmove.addOption("Get Out Of Way", "Get Out Of Way");
    //SmartDashboard.putData("First Auto Choice", autofirstmove);

    // add auton choices middle
    autoChooserPosition.setDefaultOption("Red right", Redrightauto);
    autoChooserPosition.addOption("Red middle", Redmiddleauto);
    autoChooserPosition.addOption("Red left", Redleftauto);
    autoChooserPosition.addOption("Blue right", Bluerightauto);
    autoChooserPosition.addOption("Blue middle", Bluemiddleauto);
    autoChooserPosition.addOption("Blue left", Blueleftauto);
    autoChooserPosition.addOption(" Only Shoot", onlyShoot);
    autoChooserPosition.addOption("Clockwise Exit", clockwiseExit);
    autoChooserPosition.addOption("Counter Clockwise Exit", ccwExit);
    autoChooserPosition.addOption("Middle Zero on Ground", zeroMiddle);
    SmartDashboard.putData("Autonomus Choices", autoChooserPosition);

/* 
    // final action
    autoAction.addOption("Shoot", "Shoot");
    autoAction.addOption("Stand Still", "Stand Still");
    autoAction.addOption("DriveLeftForward", "Drive Left Forward");
    autoAction.addOption("DriveRightForward", "Drive Right Forward");

    SmartDashboard.putData("Autonomus Action", autoAction);
*/
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
    SmartDashboard.putNumber("Right Drive Encoder", rightDrivetrainEncoder.getPosition());
    SmartDashboard.putNumber("Left Drive Encoder", leftDrivetrainEncoder.getPosition());
  }
  @Override
  public void teleopInit() {
      rightFrontDrivetrainMotor.setInverted(false); 
      leftRearDrivetrainMotor.set(0);
      rightRearDrivetrainMotor.set(0);
      rightFrontDrivetrainMotor.set(0);
      leftFrontDrivetrainMotor.set(0);
      setIntakeMotor(0, 25);
      setLaunchMotor(0, 25);
      rightFrontDrivetrainMotor.setIdleMode(IdleMode.kCoast);
      rightRearDrivetrainMotor.setIdleMode(IdleMode.kCoast);
      leftFrontDrivetrainMotor.setIdleMode(IdleMode.kCoast);
      leftRearDrivetrainMotor.setIdleMode(IdleMode.kCoast);
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
      turbo_multi = 1.0;
    } else if (driverController.getRawButtonPressed(3))
    {
      // B
      turbo_multi = 0.65;
    } else if (driverController.getRawButtonPressed(4))
    {
      // Y
      turbo_multi = 0.85;
    }


  drivetrain.arcadeDrive(turbo_multi * -driverController.getRawAxis(1), turbo_multi * -driverController.getRawAxis(4),true);

  if (operatorController.getRawButtonPressed(1)) {
      setIntakeMotor(.5, 25);
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
      armTarget=46.5*1.6;
    }
  if (operatorController.getRawButton(6)) { 
      armTarget=37.5*1.6;
       }
  if (operatorController.getRawButton(7)) { 
      armTarget=0;
       
      
    }
  if ((operatorController.getRawButton(8))) {
      armTarget=-8*1.6;
    
  }
    if (operatorController.getRawAxis(2)>0.7) { 
        setGroundPosition(); 
    }

    if (operatorController.getRawAxis(3)>0.7 ){
      rightArmMotor.set(0);
      leftArmMotor.set(0);
    }
    if (operatorController.getPOV()>=60 && operatorController.getPOV()<=120) armJoystick=true;
    if (operatorController.getPOV()<=300 && operatorController.getPOV()>=240) armJoystick=false;
    if ((operatorController.getPOV()>=330 && operatorController.getPOV()<=360) || (operatorController.getPOV()>=0 && operatorController.getPOV()<=30)) setLiftMotor(.3, 25);
    if (operatorController.getPOV()>=150 && operatorController.getPOV()<=210) setLiftMotor(-.5, 35);
    if (operatorController.getRawButton(10) || driverController.getRawButton(10)) armTarget=20*1.6;
    if (operatorController.getRawButton(9)) setLiftMotor(0,25);
    if (operatorController.getPOV()==-1) setLiftMotor(0, 25);
    double armAxis = Math.pow(operatorController.getRawAxis(1),3)*.3;
  
    if (armJoystick==true) {
    setArmMotor(armAxis, 25);
    }
    else{
      if (rightArmMotorEncoder.getPosition() - armTarget >=.5 || rightArmMotorEncoder.getPosition() - armTarget <=-.5 ) {armPidcontrol(armTarget);}
      else if (rightArmMotorEncoder.getPosition()<=-8*1.6) {armPidcontrol(-7*1.6);}
      else if (rightArmMotorEncoder.getPosition()>=47.5*1.6) armPidcontrol(47*1.6);

    }
    SmartDashboard.putNumber("armTarget", armTarget);

    
SmartDashboard.putBoolean("Arm Joystick", armJoystick);

  }

  double autonomousStartTime;
  public void armPidcontrol(double target) {
      rightArmMotorPID.setReference(target, CANSparkMax.ControlType.kPosition);
      leftArmMotorPID.setReference(-target, CANSparkMax.ControlType.kPosition);
  }

  public void PIDDriveTurn(double angle, double lastTimeStamp) {
    double targtAngle = angle;
    if (targtAngle - gyro.getAngle()<=3 && targtAngle-gyro.getAngle()>=-3) targtAngle=gyro.getAngle();
      double currentangle = gyro.getAngle();

      double error = targtAngle - currentangle;
      double dt = Timer.getFPGATimestamp() - lastTimeStamp;

      // accounts for friction is the error is below 1
      if (Math.abs(error) < 1) {
        errorSum += error * dt;
      }

      double errorRate = (error - lastError) / dt;

      double speedRobotRight = ((rightkP_P * error) + (rightkI_P * errorSum) + (rightkD_P * errorRate));
      double speedRobotLeft = ((leftkP_P * error) + (leftkI_P * errorSum) + (leftkD_P * errorRate));

      drivetrain.tankDrive(-speedRobotLeft, -speedRobotRight);

    if (targtAngle - gyro.getAngle()<=3 && targtAngle-gyro.getAngle()>=-3)  drivetrain.tankDrive(0, 0);


      lastTimeStamp = Timer.getFPGATimestamp();
      lastError = error;
      SmartDashboard.putNumber("Angle Target", targtAngle);
      SmartDashboard.putNumber("Special Gyro", gyro.getAngle());
    
  }
  double wheelcircumferencetofeet = (6 * Math.PI) / 12;

  public void PIDDriveFeet(double feet) {
    double leftTarget= leftDrivetrainEncoder.getPosition()+feet/wheelcircumferencetofeet;
    double rightTarget = rightDrivetrainEncoder.getPosition() - feet/wheelcircumferencetofeet;
    rightDrivetrainPID.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
    leftDrivetrainPID.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void autonomousInit() {
    rightFrontDrivetrainMotor.setInverted(true); 
    autonomousStartTime=Timer.getFPGATimestamp();
   // autofirstchoice = autofirstmove.getSelected();
    chosenauto = autoChooserPosition.getSelected();
 //   actionauto = autoAction.getSelected();
   // rightDrivetrainEncoder.setPosition(0);
    //leftDrivetrainEncoder.setPosition(0);
   // SmartDashboard.putString("Auto first choice", autofirstchoice);
    SmartDashboard.putString("chosen auto", chosenauto);
    //SmartDashboard.putString("Last action auto", actionauto);
    gyro.reset();
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
    switch (chosenauto) { // point of view for these position is from the middle of the field
      case Redrightauto:

        // go forward, turn around 130 degrees, shoot the note, turn back around, go forward, pick up note, and place it in the amp
       if (timeElapsed<=.5) { 
          setLaunchMotor(.75, 25);
          armTarget = 46.5*1.6; //changed to 37.5 was shooting too high
          armPidcontrol(armTarget);
       }
        else if (timeElapsed<=3) {
        setIntakeMotor(-1, 25);
        setIntakeMotor(.2, 25);
        setIntakeMotor(-1, 25);
        setIntakeMotor(.2, 25);
        setIntakeMotor(-1, 25);
       }
        else if (timeElapsed<=4) {
          setIntakeMotor(0, 20);
          setLaunchMotor(0, 25);
        }
        else if (timeElapsed<=5) {
          PIDDriveFeet(1);
         }
        else if (timeElapsed<=7) {
          setLaunchMotor(.7, 25);
          setIntakeMotor(-.7, 25);
        }
        else if (timeElapsed<=8) {
          setLaunchMotor(0,25);
          setIntakeMotor(0,25);
          PIDDriveTurn(-45,timeElapsed-7);
          PIDDriveFeet(3);
        }

         else {
          drivetrain.arcadeDrive(0,0);
        }
        // wait for a few seconds (idk how to wait)

        // pick up note
        break;
      case Redmiddleauto:
        if (timeElapsed<=.2) {
          setLaunchMotor(.7, 25);
          armTarget = 37.5*1.6;
          armPidcontrol(armTarget);
       }
        else if (timeElapsed<=2) {}
        else if (timeElapsed<=3) {
        setIntakeMotor(-1, 25);

       }
        else if (timeElapsed<=4) {
          setIntakeMotor(0, 20);
          setLaunchMotor(0, 25);

        }
        else if (timeElapsed<=5.5) {
          double leftTarget= leftDrivetrainEncoder.getPosition()+1/wheelcircumferencetofeet;
          double rightTarget = rightDrivetrainEncoder.getPosition() - 1/wheelcircumferencetofeet;
          rightDrivetrainPID.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
          leftDrivetrainPID.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
          armTarget = 48*1.6;
          armPidcontrol(armTarget);
          setIntakeMotor(-.3, 25);
        }
        
        else if (timeElapsed<=6) {
          setIntakeMotor(0, 25);        

        }
        else if (timeElapsed<=6.5) {
          setLaunchMotor( .7, 25);
          armTarget = 29.5*1.6;
          armPidcontrol(armTarget);
        }
        else if (timeElapsed<=8) {
          setLaunchMotor(.7, 25);
        }
        else if (timeElapsed<=9) setIntakeMotor(-.4, 25);

        else if (timeElapsed<=10) {
          setLaunchMotor(0,25);
          setIntakeMotor(0,25);
        }
        else if (timeElapsed<=11) {
          PIDDriveFeet(3.5);

        }
        else {
          drivetrain.arcadeDrive(0,0);
        }
        break;
      case Redleftauto:
        // shoot note, go forward into the middle and fire all the notes back to our base.
        if (timeElapsed<=.2) {
          setLaunchMotor(.7, 25);
          armTarget = 37.5*1.6;
          armPidcontrol(armTarget);  // check target value this seems odd__________________________________________________________________
       }
        else if (timeElapsed<=2) {}
        else if (timeElapsed<=3) {
        setIntakeMotor(-1, 25);
       }
        else if (timeElapsed<=4) {
          setIntakeMotor(0, 20);
          setLaunchMotor(0, 25);
        }
        else if (timeElapsed<=5.5) {
          double leftTarget= leftDrivetrainEncoder.getPosition()+1/wheelcircumferencetofeet;
          double rightTarget = rightDrivetrainEncoder.getPosition() - 1/wheelcircumferencetofeet;
          rightDrivetrainPID.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
          leftDrivetrainPID.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
        }
        else if (timeElapsed<=5.6) {
          lastError=0;
          errorSum=0;
          drivetrain.tankDrive(0, 0);
        }
        else if (timeElapsed <= 7) {
          PIDDriveTurn(25, timeElapsed-5.6);
        }
        else if (timeElapsed <= 9) {
          PIDDriveFeet(3);
        }
        break;
      case Bluerightauto:
        // place the note down or shoot it, go forward into the middle and fire all the notes back to our base.



        // real code
        if (timeElapsed<=2) PIDDriveFeet(1);
        else if (timeElapsed <= 3){
          setLaunchMotor(.7, 25);
          armTarget = 46.5*1.6;
          armPidcontrol(armTarget);
        }
        else if (timeElapsed <= 5) {
          setIntakeMotor(-1, 25);
          setIntakeMotor(.2, 25);
          setIntakeMotor(-1, 25);
          setIntakeMotor(.2, 25);
          setIntakeMotor(-1, 25);

          setIntakeMotor(0, 25);
          setLaunchMotor(0, 25);
        }
        else if (timeElapsed <= 7) PIDDriveFeet(-2);
        else if (timeElapsed <= 9) PIDDriveTurn(-100, timeElapsed-7);
        else if (timeElapsed <= 9.1) {
          drivetrain.tankDrive(0, 0);
        }
        else if (timeElapsed <= 12) PIDDriveFeet(5);
        break;
      case Bluemiddleauto:

        // go forward, turn 150 degrees, shoot note, turn 150 degrees back, pick up note, and shoot it into speaker.
        if (timeElapsed<=.2) {
          setLaunchMotor(.7, 25);
          armTarget = 46.5*1.6;
          armPidcontrol(armTarget);
       }
        else if (timeElapsed<=2) {}
        else if (timeElapsed<=3) {
        setIntakeMotor(-1, 25);
        setIntakeMotor(.2, 25);
        setIntakeMotor(-1, 25);
        setIntakeMotor(.2, 25);
        setIntakeMotor(-1, 25);
       }
        else if (timeElapsed<=4) {
          setIntakeMotor(0, 20);
          setLaunchMotor(0, 25);

        }
        else if (timeElapsed<=5) {
          double leftTarget= leftDrivetrainEncoder.getPosition()+1/wheelcircumferencetofeet;
          double rightTarget = rightDrivetrainEncoder.getPosition() - 1/wheelcircumferencetofeet;
          rightDrivetrainPID.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
          leftDrivetrainPID.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
          //setLaunchMotor(.5, 25);
          setIntakeMotor(-.3, 25);
        }
        else if (timeElapsed<=7) {
          armTarget = 37.5*1.6;
          armPidcontrol(armTarget);
          setLaunchMotor( .5, 25);
        }     
        else if (timeElapsed<=8) {
          setLaunchMotor(.7, 25);
        }
        else if (timeElapsed<=9) setIntakeMotor(-.7, 25);

        else if (timeElapsed<=10) {
          setLaunchMotor(0,25);
          setIntakeMotor(0,25);
          PIDDriveTurn(-35,timeElapsed-8);
        }
        else if (timeElapsed<=14.5) {
          PIDDriveFeet(3.5);

        }
        else {
          drivetrain.arcadeDrive(0,0);
        }
        break;
      case Blueleftauto:
        // go forward, turn around 130 degrees, shoot the note, turn back around, go forward, pick up note, and place it in the amp

        break;
      case onlyShoot:
         if (timeElapsed<=.91) setArmMotor(.5,20);
        else if (timeElapsed<=1.75) {
          rightArmMotor.set(0);
          leftArmMotor.set(0);
        }
        else if (timeElapsed<=2) {
          rightArmMotorEncoder.setPosition(48);
          leftArmMotorEncoder.setPosition(-48);
        }
        else if (timeElapsed<=2.25) {
          setLaunchMotor(.7, 25);
          armPidcontrol(37.5*1.6);
          // may need to adjust to 37.5
        }
        else if (timeElapsed<=4) {}
        else if (timeElapsed<=5) {
        setIntakeMotor(-1, 25);
        }
        else {
          setLaunchMotor(0, 25);
          setIntakeMotor(0, 25);
        }

        break;
      case clockwiseExit:
         if (timeElapsed<=.91) setArmMotor(.5,20);
        else if (timeElapsed<=1.75) {
          rightArmMotor.set(0);
          leftArmMotor.set(0);
        }
        else if (timeElapsed<=2) {
          rightArmMotorEncoder.setPosition(48);
          leftArmMotorEncoder.setPosition(-48);
        }
        else if (timeElapsed<=2.25) {
          setLaunchMotor(.7, 25);
          armPidcontrol(37.5*1.6);
          // may need to adjust to 37.5
        }
        else if (timeElapsed<=4) {}
        else if (timeElapsed<=5) {
        setIntakeMotor(-1, 25);
        }
        else if (timeElapsed<=6) {
          setLaunchMotor(0, 25);
          setIntakeMotor(0, 25);
        }
        else if (timeElapsed<=9) {}
        else if (timeElapsed<=10) {
          PIDDriveTurn(-35, timeElapsed-9);
        }
        else if (timeElapsed<=12.5) {
           double leftTarget= leftDrivetrainEncoder.getPosition()+1/wheelcircumferencetofeet;
          double rightTarget = rightDrivetrainEncoder.getPosition() - 1/wheelcircumferencetofeet;
          rightDrivetrainPID.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
          leftDrivetrainPID.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
        }
        break;
      case ccwExit:
         if (timeElapsed<=.91) setArmMotor(.5,20);
        else if (timeElapsed<=1.75) {
          rightArmMotor.set(0);
          leftArmMotor.set(0);
        }
        else if (timeElapsed<=2) {
          rightArmMotorEncoder.setPosition(48);
          leftArmMotorEncoder.setPosition(-48);
        }
        else if (timeElapsed<=2.25) {
          setLaunchMotor(.7, 25);
          armPidcontrol(37.5*1.6);
          // may need to adjust to 37.5
        }
        else if (timeElapsed<=4) {}
        else if (timeElapsed<=5) {
        setIntakeMotor(-1, 25);
        }
        else if (timeElapsed<=6) {
          setLaunchMotor(0, 25);
          setIntakeMotor(0, 25);
        }
        else if (timeElapsed<=9) {}
        else if (timeElapsed<=10) {
          PIDDriveTurn(35, timeElapsed-9);
        }
        else if (timeElapsed<=12.5) {
           double leftTarget= leftDrivetrainEncoder.getPosition()+1/wheelcircumferencetofeet;
          double rightTarget = rightDrivetrainEncoder.getPosition() - 1/wheelcircumferencetofeet;
          rightDrivetrainPID.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
          leftDrivetrainPID.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
        }
        break;
      case zeroMiddle:
        if (timeElapsed<=.91) setArmMotor(.5,20);
        else if (timeElapsed<=1.75) {
          rightArmMotor.set(0);
          leftArmMotor.set(0);
        }
        else if (timeElapsed<=2) {
          rightArmMotorEncoder.setPosition(48);
          leftArmMotorEncoder.setPosition(-48);
        }
        else if (timeElapsed<=2.25) {
          setLaunchMotor(.7, 25);
          armPidcontrol(37.5*1.6);
          // may need to adjust to 37.5
        }
        else if (timeElapsed<=4) {}
        else if (timeElapsed<=5) {
        setIntakeMotor(-1, 25);
       }
        else if (timeElapsed<=6) {
          setIntakeMotor(0, 20);
          setLaunchMotor(0, 25);
        }
        else if (timeElapsed<=7.5) {
          double leftTarget= leftDrivetrainEncoder.getPosition()+1/wheelcircumferencetofeet;
          double rightTarget = rightDrivetrainEncoder.getPosition() - 1/wheelcircumferencetofeet;
          rightDrivetrainPID.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
          leftDrivetrainPID.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
          //setLaunchMotor(.5, 25);
          armTarget = 48*1.6;
          armPidcontrol(armTarget);
          setIntakeMotor(-.3, 25);
        }
        
        else if (timeElapsed<=8) {
          setIntakeMotor(0, 25);        

        }
        else if (timeElapsed<=8.5) {
          setLaunchMotor( .7, 25);
          armTarget = 29.5*1.6;
          armPidcontrol(armTarget);
        }
        else if (timeElapsed<=10) {
          setLaunchMotor(.7, 25);
        }
        else if (timeElapsed<=11) setIntakeMotor(-.4, 25);

        else if (timeElapsed<=12) {
          setLaunchMotor(0,25);
          setIntakeMotor(0,25);
         // PIDDriveTurn(35,timeElapsed-8);
        }
        else if (timeElapsed<=13) {
          PIDDriveFeet(3.5);

        }
        else {
          drivetrain.arcadeDrive(0,0);
        }
        break;
      default:
        break;
    }
    }

 
  @Override
  public void disabledInit() {
    //rightDrivetrainEncoder.setPosition(0);
    //leftDrivetrainEncoder.setPosition(0);
   //leftArmMotor.setIdleMode(IdleMode.kCoast);
   // rightArmMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {}

  public void setGroundPosition() {
      rightArmMotorEncoder.setPosition(48*1.6);
      leftArmMotorEncoder.setPosition(-48*1.6);
  

    
  }
  
}
 