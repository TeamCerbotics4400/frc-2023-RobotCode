// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax flyWheelLower = new CANSparkMax(ShooterConstants.FLY_WHEEL_LOWER_ID, MotorType.kBrushless);
  private CANSparkMax flyWheelUpper = new CANSparkMax(ShooterConstants.FLY_WHEEL_UPPER_ID, MotorType.kBrushless);

  private double targetVelocity = 0;
  private RelativeEncoder flyWheelLowerEncoder = flyWheelLower.getEncoder();
  private RelativeEncoder flyWheelUpperEncoder = flyWheelUpper.getEncoder();

  private SparkMaxPIDController flyWheelLower_PIDController = flyWheelLower.getPIDController();
  private SparkMaxPIDController flyWheelUpper_PIDController = flyWheelUpper.getPIDController();

  public Shooter() {
    flyWheelLower.restoreFactoryDefaults();
    flyWheelUpper.restoreFactoryDefaults();

    flyWheelLower.setInverted(true);
    flyWheelUpper.setInverted(false);

    flyWheelLower.setCANTimeout(10);
    flyWheelUpper.setCANTimeout(10);

    flyWheelLower.setIdleMode(IdleMode.kCoast);
    flyWheelUpper.setIdleMode(IdleMode.kCoast);

    int smartMotionSlot = 0;
    flyWheelLower_PIDController.setSmartMotionMaxVelocity(ShooterConstants.maxVel, smartMotionSlot);
    flyWheelLower_PIDController.setSmartMotionMinOutputVelocity(ShooterConstants.minVel, smartMotionSlot);
    flyWheelLower_PIDController.setSmartMotionMaxAccel(ShooterConstants.maxAcc, smartMotionSlot);
    flyWheelLower_PIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.allowedErr, smartMotionSlot);

    flyWheelUpper_PIDController.setSmartMotionMaxVelocity(ShooterConstants.maxVel, smartMotionSlot);
    flyWheelUpper_PIDController.setSmartMotionMinOutputVelocity(ShooterConstants.minVel, smartMotionSlot);
    flyWheelUpper_PIDController.setSmartMotionMaxAccel(ShooterConstants.maxAcc, smartMotionSlot);
    flyWheelUpper_PIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.allowedErr, smartMotionSlot);

    SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
    SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
    SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ShooterConstants.kFF);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
    SmartDashboard.putNumber("Target Velocity", targetVelocity);

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
    double targetVelo = SmartDashboard.getNumber("Target Velocity", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != ShooterConstants.kP)) {   flyWheelLower_PIDController.setP(p);     flyWheelUpper_PIDController.setP(p);      ShooterConstants.kP = p; }
    if((i != ShooterConstants.kI)) {   flyWheelLower_PIDController.setI(i);     flyWheelUpper_PIDController.setI(i);      ShooterConstants.kI = i; }
    if((d != ShooterConstants.kD)) {   flyWheelLower_PIDController.setD(d);     flyWheelUpper_PIDController.setD(d);      ShooterConstants.kD = d; }
    if((iz != ShooterConstants.kIz)) { flyWheelLower_PIDController.setIZone(iz);flyWheelUpper_PIDController.setIZone(iz); ShooterConstants.kIz = iz; }
    if((ff != ShooterConstants.kFF)) { flyWheelLower_PIDController.setFF(ff);   flyWheelUpper_PIDController.setFF(ff);    ShooterConstants.kFF = ff; }
    if ((targetVelocity != targetVelo )) {targetVelocity = targetVelo;};
    if((max != ShooterConstants.kMaxOutput) || (min != ShooterConstants.kMinOutput)) { 
      flyWheelLower_PIDController.setOutputRange(min, max); 
      ShooterConstants.kMinOutput = min; ShooterConstants.kMaxOutput = max; 
    }
    if((maxV != ShooterConstants.maxVel)) { flyWheelLower_PIDController.setSmartMotionMaxVelocity(maxV,0); ShooterConstants.maxVel = maxV; }
    if((minV != ShooterConstants.minVel)) { flyWheelLower_PIDController.setSmartMotionMinOutputVelocity(minV,0); ShooterConstants.minVel = minV; }
    if((maxA != ShooterConstants.maxAcc)) { flyWheelLower_PIDController.setSmartMotionMaxAccel(maxA,0); ShooterConstants.maxAcc = maxA; }
    if((allE != ShooterConstants.allowedErr)) { flyWheelLower_PIDController.setSmartMotionAllowedClosedLoopError(allE,0); ShooterConstants.allowedErr = allE; }

  }

  public void setLowerFlyWheelVelo(double setPoint){
    flyWheelLower_PIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }
  public void setUpperFlyWheelVelo(double setPoint){
    flyWheelUpper_PIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void goToDashboardVelocity(){
    setLowerFlyWheelVelo(targetVelocity);
    setUpperFlyWheelVelo(targetVelocity);
  }

  public void setMotorsPower(double UpperPower, double LowerPower){
    flyWheelLower.set(LowerPower);
    flyWheelUpper.set(UpperPower);
  }
}