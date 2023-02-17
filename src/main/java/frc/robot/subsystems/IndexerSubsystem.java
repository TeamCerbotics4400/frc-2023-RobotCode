// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new FalconShooter. */
  CANSparkMax lowerLeftFlywheel = new CANSparkMax(ShooterConstants.LOWER_LEFT_FLY, MotorType.kBrushless);
  CANSparkMax lowerRightFlywheel = new CANSparkMax(ShooterConstants.LOWER_RIGHT_FLY, MotorType.kBrushless);

  SparkMaxPIDController lowerLeftController = lowerLeftFlywheel.getPIDController();
  SparkMaxPIDController lowerRightController = lowerRightFlywheel.getPIDController();

  RelativeEncoder lowerLeftEncoder = lowerLeftFlywheel.getEncoder();
  RelativeEncoder lowerRightEncoder = lowerRightFlywheel.getEncoder();

  double desiredVelo = 0;

  int pidSlot = 0;

  public IndexerSubsystem() {

    lowerLeftFlywheel.restoreFactoryDefaults();
    lowerRightFlywheel.restoreFactoryDefaults();

    lowerLeftFlywheel.setInverted(false);
    lowerRightFlywheel.setInverted(true);

    lowerLeftFlywheel.setIdleMode(IdleMode.kCoast);
    lowerRightFlywheel.setIdleMode(IdleMode.kCoast);

    lowerLeftController.setP(ShooterConstants.LkP);
    lowerLeftController.setI(ShooterConstants.LkI);
    lowerLeftController.setD(ShooterConstants.LkD);
    lowerLeftController.setFF(ShooterConstants.LkFF);

    lowerRightController.setP(ShooterConstants.LkP);
    lowerRightController.setI(ShooterConstants.LkI);
    lowerRightController.setD(ShooterConstants.LkD);
    lowerRightController.setFF(ShooterConstants.LkFF);
    
    SmartDashboard.putNumber("Target Velo", desiredVelo);
/* 
    SmartDashboard.putNumber("Shooter P", ShooterConstants.kP);
    SmartDashboard.putNumber("Shooter I", ShooterConstants.kI);
    SmartDashboard.putNumber("Shooter D", ShooterConstants.kD);
    SmartDashboard.putNumber("Shooter Iz", ShooterConstants.kIz);
    SmartDashboard.putNumber("Shooter FF", ShooterConstants.kFF);*/

    SmartDashboard.putNumber("Lower P", ShooterConstants.LkP);
    SmartDashboard.putNumber("Lower I", ShooterConstants.LkI);
    SmartDashboard.putNumber("Lower D", ShooterConstants.LkD);
    SmartDashboard.putNumber("Lower FF", ShooterConstants.LkFF);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Avergae RPM NEO", getAverageNeoRPM());

    double targetVelo = SmartDashboard.getNumber("Target Velo", 0);

    if((desiredVelo != targetVelo)){desiredVelo = targetVelo;}

    double lP = SmartDashboard.getNumber("Lower P", ShooterConstants.LkP);
    double lI = SmartDashboard.getNumber("Lower I", ShooterConstants.LkI);
    double lD = SmartDashboard.getNumber("Lower D", ShooterConstants.LkD);
    double lFF = SmartDashboard.getNumber("Lower FF", ShooterConstants.LkFF);

    if((lP != ShooterConstants.LkP)){lowerLeftController.setP(lP); lowerRightController.setP(lP);
                                      ShooterConstants.LkP = lP;}
    if((lI != ShooterConstants.LkI)){lowerLeftController.setI(lI); lowerRightController.setI(lI);
                                      ShooterConstants.LkI = lI;}
    if((lD != ShooterConstants.LkD)){lowerLeftController.setD(lD); lowerRightController.setD(lD);
                                      ShooterConstants.LkD = lD;}
    if((lFF != ShooterConstants.LkFF)){lowerLeftController.setFF(lFF); lowerRightController.setFF(lFF);
                                        ShooterConstants.LkFF = lFF;}
    

  /* 
    double p = SmartDashboard.getNumber("Shooter P", ShooterConstants.kP);
    double i = SmartDashboard.getNumber("Shooter I", ShooterConstants.kI);
    double iz = SmartDashboard.getNumber("Shooter Iz", ShooterConstants.kIz);
    double d = SmartDashboard.getNumber("Shooter D", ShooterConstants.kD);
    double ff = SmartDashboard.getNumber("Shooter FF", ShooterConstants.kFF);

    if((p != ShooterConstants.kP)){leftFlyWheel.config_kP(pidSlot, p); 
                                    rightFlyWheel.config_kP(pidSlot,p); ShooterConstants.kP = p;}
    if((i != ShooterConstants.kI)){leftFlyWheel.config_kI(pidSlot, i); 
                                    rightFlyWheel.config_kI(pidSlot, i); ShooterConstants.kI = i;}
    if((iz != ShooterConstants.kIz)){leftFlyWheel.config_IntegralZone(pidSlot, iz); 
                                    rightFlyWheel.config_IntegralZone(pidSlot, iz); ShooterConstants.kIz = iz;}                                
    if((d != ShooterConstants.kD)){leftFlyWheel.config_kD(pidSlot, d); 
                                    rightFlyWheel.config_kD(pidSlot,d); ShooterConstants.kD = d;}
    if((ff != ShooterConstants.kFF)){leftFlyWheel.config_kF(pidSlot, ff); 
                                    rightFlyWheel.config_kF(pidSlot,ff); ShooterConstants.kFF = ff;}*/
  }

  public double getLowerLeftRPM(){
    return lowerLeftEncoder.getVelocity();
  }
  
  public double getLowerRightRPM(){
    return lowerRightEncoder.getVelocity();
  }

  public double getAverageNeoRPM(){
    return (getLowerLeftRPM() + getLowerRightRPM()) / 2;
  }

  public double getTargetVelo(){
    return desiredVelo;
  }

  /*public boolean isOnTarget(){
    double rpmDifference = getTargetVelo() - getAverageRPM();

    if(rpmDifference <= ShooterConstants.shooterTreshold){
      return onTarget = true;
    } else {
      return onTarget = false;
    }
  }*/

  public void lowerLeftSetpoint(double setPoint){
    lowerLeftController.setReference(setPoint, ControlType.kVelocity);
  }

  public void lowerRightSetpoint(double setPoint){
    lowerRightController.setReference(setPoint, ControlType.kVelocity);
  }

  public void setNeoVelo(double leftVelo, double rightVelo){
    lowerLeftSetpoint(leftVelo);
    lowerRightSetpoint(rightVelo);
  }

  public void goToDashboardVelocity(){
    lowerLeftSetpoint(desiredVelo);
    lowerRightSetpoint(desiredVelo);
  }

  public void setMotorsPower(double leftPower, double rightPower){
    lowerLeftFlywheel.set(leftPower);
    lowerRightFlywheel.set(rightPower);
  }
}
