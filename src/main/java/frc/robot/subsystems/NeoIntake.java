// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
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
  /** Creates a new NeoIntake. 

  CANSparkMax RPWheelIntake = new CANSparkMax(IntakeConstants.RapidWheeel_ID, MotorType.kBrushless);
  CANSparkMax I_Should_Be_A_Servo = new CANSparkMax(IntakeConstants.IShouldBeAServo_ID, MotorType.kBrushless);

  private RelativeEncoder RPWheelEncoder =  RPWheelIntake.getEncoder();
  

  private SparkMaxPIDController RPWheelIntake_PIDController = RPWheelIntake.getPIDController();
  private SparkMaxPIDController ishouldbeaservo_PIDController = I_Should_Be_A_Servo.getPIDController();

  private double targetVelocity = 0;

  public NeoIntake() { 
    RPWheelIntake.restoreFactoryDefaults();
    I_Should_Be_A_Servo.restoreFactoryDefaults();

    RPWheelIntake.setInverted(true);
    I_Should_Be_A_Servo.setInverted(true);

    RPWheelIntake.setCANTimeout(10);
    I_Should_Be_A_Servo.setCANTimeout(10);

    RPWheelIntake.setIdleMode(IdleMode.kCoast);
    I_Should_Be_A_Servo.setIdleMode(IdleMode.kCoast);

    int smartMotionSlot = 0;

    RPWheelIntake_PIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    ishouldbeaservo_PIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    ishouldbeaservo_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    ishouldbeaservo_PIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    ishouldbeaservo_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    SmartDashboard.putNumber("P Gain", IntakeConstants.kP);
    SmartDashboard.putNumber("I Gain", IntakeConstants.kI);
    SmartDashboard.putNumber("D Gain", IntakeConstants.kD);
    SmartDashboard.putNumber("I Zone", IntakeConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", IntakeConstants.kFF);
    SmartDashboard.putNumber("Max Output", IntakeConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", IntakeConstants.kMinOutput);
    SmartDashboard.putNumber("Target Velocity", targetVelocity);

    SmartDashboard.putNumber("Target INTAKE Velocity", IntakeConstants.targetVelocity);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P INTAKE Gain", 0);
    double i = SmartDashboard.getNumber("I INTAKE Gain", 0);
    double d = SmartDashboard.getNumber("D INTAKE Gain", 0);
    double iz = SmartDashboard.getNumber("I INTAKE Zone", 0);
    double ff = SmartDashboard.getNumber("Feed INTAKE Forward", 0);
    double max = SmartDashboard.getNumber("Max INTAKE Output", 0);
    double min = SmartDashboard.getNumber("Min INTAKE Output", 0);
    double maxV = SmartDashboard.getNumber("Max INTAKE Velocity", 0);
    double minV = SmartDashboard.getNumber("Min INTAKE Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max INTAKE Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop INTAKE Error", 0);
    double targetVelo = SmartDashboard.getNumber("Target INTAKE Velocity", 0);

    SmartDashboard.putNumber("Velocidad del RPwheel", RPWheelEncoder.getVelocity());


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != IntakeConstants.kP)) {   RPWheelIntake_PIDController.setP(p);     }
    if((i != IntakeConstants.kI)) {   RPWheelIntake_PIDController.setI(i);     }
    if((d != IntakeConstants.kD)) {   RPWheelIntake_PIDController.setD(d);     }
    if((iz != IntakeConstants.kIz)) { RPWheelIntake_PIDController.setIZone(iz);}
    if((ff != IntakeConstants.kFF)) { RPWheelIntake_PIDController.setFF(ff);   }
    if ((targetVelocity != targetVelo )) {targetVelocity = targetVelo;};
    if((max != IntakeConstants.kMaxOutput) || (min != IntakeConstants.kMinOutput)) { 
      RPWheelIntake_PIDController.setOutputRange(min, max); 
      IntakeConstants.kMinOutput = min; IntakeConstants.kMaxOutput = max; 
    }
    if((maxV != IntakeConstants.maxVel)) { RPWheelIntake_PIDController.setSmartMotionMaxVelocity(maxV,0); IntakeConstants.maxVel = maxV; }
    if((minV != IntakeConstants.minVel)) { RPWheelIntake_PIDController.setSmartMotionMinOutputVelocity(minV,0); IntakeConstants.minVel = minV; }
    if((maxA != IntakeConstants.maxAcc)) { RPWheelIntake_PIDController.setSmartMotionMaxAccel(maxA,0); IntakeConstants.maxAcc = maxA; }
    if((allE != IntakeConstants.allowedErr)) { RPWheelIntake_PIDController.setSmartMotionAllowedClosedLoopError(allE,0); IntakeConstants.allowedErr = allE; }


  }

  public void SetRapidWheelVelocity(double setPoint){
    RPWheelIntake_PIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }


  public void goToDashboardVelocity(){
    SetRapidWheelVelocity(targetVelocity);
    SetRapidWheelVelocity(IntakeConstants.targetVelocity);

  }

  public void setMotorsPower(double Power){
    RPWheelIntake.set(Power);
  }
}*/
