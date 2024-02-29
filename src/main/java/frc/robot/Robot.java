package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Robot extends TimedRobot {
 // private static final int deviceID = 12;
  private CANSparkMax m_motor12;
  private CANSparkMax m_motor13;
  private SparkPIDController m_pidController12;
  //private SparkPIDController m_pidController13;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  @Override
  public void robotInit() {
    // initialize motor
    m_motor12 = new CANSparkMax(12, MotorType.kBrushed);
    m_motor13 = new CANSparkMax(13, MotorType.kBrushed);

    m_motor13.follow(m_motor12);

    m_pidController12 = m_motor12.getPIDController();
    // m_pidController12 = m_motor13.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor12.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8092);

    // PID coefficients
    kP = 1; 
    kI = 0;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController12.setP(kP);
    m_pidController12.setI(kI);
    m_pidController12.setD(kD);
    m_pidController12.setIZone(kIz);
    m_pidController12.setFF(kFF);
    m_pidController12.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { m_pidController12.setP(p); kP = p; }
    if((p != kP)) { m_pidController12.setP(p); kP = p; }
    if((i != kI)) { m_pidController12.setI(i); kI = i; }
    if((d != kD)) { m_pidController12.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController12.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController12.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController12.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    
    m_pidController12.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
}
