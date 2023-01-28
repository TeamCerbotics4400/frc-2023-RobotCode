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
   /* Creates a new Shooter. */
  private CANSparkMax flyWheelLower = new CANSparkMax(ShooterConstants.FLY_WHEEL_LOWER_ID, MotorType.kBrushless);
  private CANSparkMax flyWheelUpper = new CANSparkMax(ShooterConstants.FLY_WHEEL_UPPER_ID, MotorType.kBrushless);
  
  private RelativeEncoder flyWheelLowerEncoder = flyWheelLower.getEncoder();
  private RelativeEncoder flyWheelUpperEncoder = flyWheelUpper.getEncoder();

  private SparkMaxPIDController flyWheelLower_PIDController = flyWheelLower.getPIDController();
  private SparkMaxPIDController flyWheelUpper_PIDController = flyWheelUpper.getPIDController();

  private double targetVelocity = 0;

  public Shooter() {
    flyWheelLower.restoreFactoryDefaults();
    flyWheelUpper.restoreFactoryDefaults();

    flyWheelLower.setInverted(true);
    flyWheelUpper.setInverted(true);

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

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double targetVelo = SmartDashboard.getNumber("Target Velocity", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((targetVelocity != targetVelo )) {targetVelocity = targetVelo;}
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