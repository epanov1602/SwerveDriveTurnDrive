package frc.robot.subsystems;

import static com.revrobotics.SparkLimitSwitch.Type.kNormallyClosed;
import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;
import static frc.robot.Constants.ExtensionConstants.initialAllowedError;
import static frc.robot.Constants.ExtensionConstants.initialD;
import static frc.robot.Constants.ExtensionConstants.initialFF;
import static frc.robot.Constants.ExtensionConstants.initialI;
import static frc.robot.Constants.ExtensionConstants.initialIz;
import static frc.robot.Constants.ExtensionConstants.initialMaxAcc;
import static frc.robot.Constants.ExtensionConstants.initialMaxInches;
import static frc.robot.Constants.ExtensionConstants.initialMaxOutput;
import static frc.robot.Constants.ExtensionConstants.initialMaxRPM;
import static frc.robot.Constants.ExtensionConstants.initialMaxVel;
import static frc.robot.Constants.ExtensionConstants.initialMinOutput;
import static frc.robot.Constants.ExtensionConstants.initialMinVel;
import static frc.robot.Constants.ExtensionConstants.initialP;
import static frc.robot.Constants.ExtensionConstants.motorRevolutionsPerInch;
import static frc.robot.Constants.ExtensionConstants.TravelMode.Velocity;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ExtensionConstants.TravelMode;

public class SmartMotionExtension extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkPIDController pidController;
  private RelativeEncoder m_encoder;
  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
  public double kP, kI, kD, kIz, kFF, maxOutput, minOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, maxInches;
  private double inchesGoal, revolutionsGoal, velocityGoal = 0;
  public TravelMode TravelMode;
  private boolean inchesMode = true;
  /**
   * Length of arm when fully retracted, in inches
   */
  public final double RetractedLength = 36;

  public double getInchesGoal() {
    return inchesGoal;
  }

  public double getRevolutionsGoal() {
    return revolutionsGoal;
  }

  public double getVelocityGoal() {
    return velocityGoal;
  }

  /*
   * Set the position goal in inches, >= 0
   */
  public void setInchesGoal(double inches) {
    System.out.println("ExtensionInchesGoal:"+inches);
    if(!IsInchesGoalValid(inches))return;
    inchesGoal = inches;
    //  SmartDashboard.putNumber("Inches Goal", inches);
    //  SmartDashboard.putNumber("Revolutions Goal", inches * motorRevolutionsPerInch);
  }

  public boolean IsInchesGoalValid(double inches){
    if (inches < 0 || inches > maxInches) {
      System.out.println("Illegal Position Goal: " + inches);
      return false;
    }
    return true;
  }

  /*
   * Set the position goal in motor revolutions
   */
  public void setRevolutionsGoal(double revs) {
    if (revs < 0 || (revs / motorRevolutionsPerInch) > maxInches) {
      System.out.println("Illegal Revolutions Goal: " + revs);
      return;
    }
    // TravelMode = Position;
    revolutionsGoal = revs;
    // SmartDashboard.putNumber("Inches Goal", revs / motorRevolutionsPerInch);
    // SmartDashboard.putNumber("Revolutions Goal", revs);
  }

  /*
   * Set the veloicty goal in motor RPM
   */
  public void setVelocityGoal(double rpm) {
    if (rpm == velocityGoal)
      return;
    if (rpm < minVel || rpm > maxVel) {
      System.out.println("Illegal Velocity Goal: " + rpm);
      return;
    }
    // TravelMode = Velocity;
    this.velocityGoal = rpm;
  }

  /** Creates a new SmartMotionElevator. */
  public SmartMotionExtension() {
    m_motor = new CANSparkMax(Constants.CANIDs.kExtensionMotor, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(false);
    m_motor.setIdleMode(IdleMode.kBrake);
    
    // configure limit switches
    m_forwardLimit = m_motor.getForwardLimitSwitch(kNormallyClosed);
    m_reverseLimit = m_motor.getReverseLimitSwitch(kNormallyClosed);
    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);

    // initialze PID controller and encoder objects
    pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);

    // PID coefficients
    kP = initialP;
    kI = initialI;
    kD = initialD;
    kIz = initialIz;
    kFF = initialFF;
    maxOutput = initialMaxOutput;
    minOutput = initialMinOutput;
    maxRPM = initialMaxRPM;

    // Smart Motion Coefficients
    maxVel = initialMaxVel; // rpm
    minVel = initialMinVel; // rpm
    maxAcc = initialMaxAcc;
    allowedErr = initialAllowedError;
    maxInches = initialMaxInches;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(minOutput, maxOutput);

    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", maxOutput);
    // SmartDashboard.putNumber("Min Output", minOutput);

    // display Smart Motion coefficients
    // SmartDashboard.putNumber("Max Velocity", maxVel);
    // SmartDashboard.putNumber("Min Velocity", minVel);
    // SmartDashboard.putNumber("Max Acceleration", maxAcc);
    // SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);

    // set Goals
    // SmartDashboard.putNumber("Revolutions Goal", 0); // motor revolutions
    // SmartDashboard.putNumber("Inches Goal", 0); // vertical inches
    // SmartDashboard.putNumber("Velocity Goal", 0);

    // button to toggle between velocity and smart motion modes
    //SmartDashboard.putBoolean("Velocity Mode", TravelMode == Velocity);

    // button to toggle between position (revolutions) and inches input
    //SmartDashboard.putBoolean("Inches Mode", inchesMode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    // double minV = SmartDashboard.getNumber("Min Velocity", 0);
    // double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    // double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if ((p != kP)) {
    // pidController.setP(p);
    // kP = p;
    // }
    // if ((i != kI)) {
    // pidController.setI(i);
    // kI = i;
    // }
    // if ((d != kD)) {
    // pidController.setD(d);
    // kD = d;
    // }
    // if ((iz != kIz)) {
    // pidController.setIZone(iz);
    // kIz = iz;
    // }
    // if ((ff != kFF)) {
    // pidController.setFF(ff);
    // kFF = ff;
    // }
    // if ((max != maxOutput) || (min != minOutput)) {
    // pidController.setOutputRange(min, max);
    // minOutput = min;
    // maxOutput = max;
    // }
    // if ((maxV != maxVel)) {
    // pidController.setSmartMotionMaxVelocity(maxV, 0);
    // maxVel = maxV;
    // }
    // if ((minV != minVel)) {
    // pidController.setSmartMotionMinOutputVelocity(minV, 0);
    // minVel = minV;
    // }
    // if ((maxA != maxAcc)) {
    // pidController.setSmartMotionMaxAccel(maxA, 0);
    // maxAcc = maxA;
    // }
    // if ((allE != allowedErr)) {
    // pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
    // allowedErr = allE;
    // }

    // double sbVgoal = SmartDashboard.getNumber("Velocity Goal", 0);
    // if (sbVgoal != getVelocityGoal()) {
    // setVelocityGoal(sbVgoal); //validate and set velocity goal
    // SmartDashboard.putNumber("Velocity Goal", getVelocityGoal());
    // }

    // double sbAgoal = SmartDashboard.getNumber("Inches Goal", 0);
    // if(sbAgoal!=getInchesGoal()){
    // setInchesGoal(sbAgoal); //validate and set inches goal
    // SmartDashboard.putNumber("Inches Goal", sbAgoal);
    // }

    // double sbRgoal = SmartDashboard.getNumber("Revolutions Goal", 0);
    // if(sbRgoal!=getRevolutionsGoal()){
    // setRevolutionsGoal(sbRgoal); //validate and set inches goal
    // SmartDashboard.putNumber("Revolutions Goal", sbRgoal);
    // }

    double setPoint, processVariable;
    // boolean mode = SmartDashboard.getBoolean("Velocity Mode", false);
    // if (mode) {
    // setPoint = getVelocityGoal();
    // pidController.setReference(getVelocityGoal(),
    // CANSparkMax.ControlType.kVelocity);
    // processVariable = m_encoder.getVelocity();
    // } else { // smart motion
    // check which input to read position or inches
    // if (SmartDashboard.getBoolean("Inches Mode", false)) {
    setPoint = getInchesGoal() * motorRevolutionsPerInch;
    // SmartDashboard.putNumber("Revolutions Goal", setPoint);
    // } else {
    //setPoint = getRevolutionsGoal();
    // SmartDashboard.putNumber("Inches Goal", setPoint / motorRevolutionsPerInch);
    // }
    pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    processVariable = m_encoder.getPosition();
    //}

    // SmartDashboard.putNumber("SetPoint", setPoint);
    // SmartDashboard.putNumber("Process Variable", processVariable);
    // SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
    // SmartDashboard.putNumber("Revolutions", m_encoder.getPosition());
    // SmartDashboard.putNumber("Inches", m_encoder.getPosition() / motorRevolutionsPerInch);
    // SmartDashboard.putNumber("Velocity", m_encoder.getVelocity());
    // SmartDashboard.putBoolean("Fwd Limit", m_forwardLimit.isPressed());
    // SmartDashboard.putBoolean("Rev Limit", m_reverseLimit.isPressed());
  }

  /*
   * Must be called before using elevator, preferably from robotInit.
   */
  public void initialize() {
    FindHome();
  }

  /*
   * Moves onto the Home switch and just off of it
   */
  public void FindHome() {
  }

  public void Stop() {
    m_motor.stopMotor();
  }

  /**
   * Get the current status of the Forward and Reverse limit switches
   * 
   * @return boolean[]{Fwd, Rev}
   */
  public boolean[] getLimitSwitches() {
    return new boolean[] { m_forwardLimit.isPressed(), m_reverseLimit.isPressed() };
  }

  public void resetEncoders() {
    m_encoder.setPosition(0);
  }

  /**
   * @return Total current length of arm, in inches
   */
public double getTotalLength() {
    return  (m_encoder.getPosition() / motorRevolutionsPerInch) + RetractedLength;
}

}
