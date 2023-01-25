// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class NeoIntake extends SubsystemBase {
  // Creates a new NeoIntake. 

  CANSparkMax I_Should_Be_A_Servo = new CANSparkMax(IntakeConstants.IShouldBeAServo_ID, MotorType.kBrushless);
  CANSparkMax RapidWheel = new CANSparkMax(IntakeConstants.RapidWheeel_ID, MotorType.kBrushless);

  private RelativeEncoder i_Should_Be_A_Servo_Encoder =  I_Should_Be_A_Servo.getEncoder();
  

  private SparkMaxPIDController I_Should_Be_A_Servo_PIDController = I_Should_Be_A_Servo.getPIDController();
  private SparkMaxPIDController RapidWHeelPIDController = RapidWheel.getPIDController();

  private double targetPosition = 0;
  private double RapidVelo = 0;

  public NeoIntake() { 
    I_Should_Be_A_Servo.restoreFactoryDefaults();
    RapidWheel.restoreFactoryDefaults();

    I_Should_Be_A_Servo.setInverted(true);
    RapidWheel.setInverted(true);

    I_Should_Be_A_Servo.setCANTimeout(10);
    RapidWheel.setCANTimeout(10);

    I_Should_Be_A_Servo.setIdleMode(IdleMode.kBrake);
    RapidWheel.setIdleMode(IdleMode.kBrake);

    int smartMotionSlot = 0;

    I_Should_Be_A_Servo_PIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    I_Should_Be_A_Servo_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    I_Should_Be_A_Servo_PIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    I_Should_Be_A_Servo_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    RapidWHeelPIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    RapidWHeelPIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    RapidWHeelPIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    RapidWHeelPIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    SmartDashboard.putNumber("P Gain", IntakeConstants.kP);
    SmartDashboard.putNumber("I Gain", IntakeConstants.kI);
    SmartDashboard.putNumber("D Gain", IntakeConstants.kD);
    SmartDashboard.putNumber("I Zone", IntakeConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", IntakeConstants.kFF);
    SmartDashboard.putNumber("Max Output", IntakeConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", IntakeConstants.kMinOutput);
    SmartDashboard.putNumber("RP Power", RapidVelo);
    SmartDashboard.putNumber("Set Rotations", targetPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    double RapidWheelVelo = SmartDashboard.getNumber("Target RapidWheel Velocity", 0);
    SmartDashboard.putNumber("SetPoint", rotations); 
    SmartDashboard.putNumber("ProcessVariable", i_Should_Be_A_Servo_Encoder.getPosition());
 

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != IntakeConstants.kP)) {   I_Should_Be_A_Servo_PIDController.setP(p);     }
    if((i != IntakeConstants.kI)) {   I_Should_Be_A_Servo_PIDController.setI(i);     }
    if((d != IntakeConstants.kD)) {   I_Should_Be_A_Servo_PIDController.setD(d);     }
    if((iz != IntakeConstants.kIz)) { I_Should_Be_A_Servo_PIDController.setIZone(iz);}
    if((ff != IntakeConstants.kFF)) { I_Should_Be_A_Servo_PIDController.setFF(ff);   }
    if ((targetPosition != rotations )) {targetPosition = rotations;}
    if ((RapidWheelVelo != RapidVelo)) {RapidWheelVelo = RapidVelo;}
    if((max != IntakeConstants.kMaxOutput) || (min != IntakeConstants.kMinOutput)) { 
      I_Should_Be_A_Servo_PIDController.setOutputRange(min, max); 
      IntakeConstants.kMinOutput = min; IntakeConstants.kMaxOutput = max; 
    }
    
    if((maxV != IntakeConstants.maxVel)) { I_Should_Be_A_Servo_PIDController.setSmartMotionMaxVelocity(maxV,0); IntakeConstants.maxVel = maxV; }
    if((minV != IntakeConstants.minVel)) { I_Should_Be_A_Servo_PIDController.setSmartMotionMinOutputVelocity(minV,0); IntakeConstants.minVel = minV; }
    if((maxA != IntakeConstants.maxAcc)) { I_Should_Be_A_Servo_PIDController.setSmartMotionMaxAccel(maxA,0); IntakeConstants.maxAcc = maxA; }
    if((allE != IntakeConstants.allowedErr)) { I_Should_Be_A_Servo_PIDController.setSmartMotionAllowedClosedLoopError(allE,0); IntakeConstants.allowedErr = allE; }
    

  }

  public void SetRapidWheelVelocity(double setPoint){
    I_Should_Be_A_Servo_PIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void goToDashboardVelocity(){
    SetRapidWheelVelocity(targetPosition);
    setMotorsPower(RapidVelo);
  }

  public void setMotorsPower(double Power){
    I_Should_Be_A_Servo.set(Power);
  }
}
