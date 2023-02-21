// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FalconShooter extends SubsystemBase {
  /** Creates a new FalconShooter. */
  TalonFX leftFlyWheel = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_ID);
  TalonFX rightFlyWheel = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_ID);

  boolean onTarget = false;

  double desiredVelo = 0;

  int pidSlot = 0;

  public FalconShooter() {
    leftFlyWheel.configFactoryDefault();
    rightFlyWheel.configFactoryDefault();

    leftFlyWheel.setInverted(false);
    rightFlyWheel.setInverted(true);

    leftFlyWheel.setNeutralMode(NeutralMode.Coast);
    rightFlyWheel.setNeutralMode(NeutralMode.Coast);

    leftFlyWheel.configVoltageCompSaturation(12);
    rightFlyWheel.configVoltageCompSaturation(12);

    leftFlyWheel.config_kP(pidSlot, ShooterConstants.kP);
    leftFlyWheel.config_kI(pidSlot, ShooterConstants.kI);
    leftFlyWheel.config_IntegralZone(pidSlot, ShooterConstants.kIz);
    leftFlyWheel.config_kD(pidSlot, ShooterConstants.kD);
    leftFlyWheel.config_kF(pidSlot, ShooterConstants.kFF);

    rightFlyWheel.config_kP(pidSlot, ShooterConstants.kP);
    rightFlyWheel.config_kI(pidSlot, ShooterConstants.kI);
    rightFlyWheel.config_kD(pidSlot, ShooterConstants.kD);
    rightFlyWheel.config_IntegralZone(pidSlot, ShooterConstants.kIz);
    rightFlyWheel.config_kF(pidSlot, ShooterConstants.kFF);
    
    SmartDashboard.putNumber("Target Velo", desiredVelo);
/* 
    SmartDashboard.putNumber("Shooter P", ShooterConstants.kP);
    SmartDashboard.putNumber("Shooter I", ShooterConstants.kI);
    SmartDashboard.putNumber("Shooter D", ShooterConstants.kD);
    SmartDashboard.putNumber("Shooter Iz", ShooterConstants.kIz);
    SmartDashboard.putNumber("Shooter FF", ShooterConstants.kFF);*/

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("Average RPM", getAverageRPM());
    //SmartDashboard.putNumber("Left RPM", getLeftRPM());
    //SmartDashboard.putNumber("Right RPM", getRightRPM());

    double targetVelo = SmartDashboard.getNumber("Target Velo", 0);

    if((desiredVelo != targetVelo)){desiredVelo = targetVelo;}

    double lP = SmartDashboard.getNumber("Lower P", ShooterConstants.LkP);
    double lI = SmartDashboard.getNumber("Lower I", ShooterConstants.LkI);
    double lD = SmartDashboard.getNumber("Lower D", ShooterConstants.LkD);
    double lFF = SmartDashboard.getNumber("Lower FF", ShooterConstants.LkFF);
    

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

  public double falconUnitsToRPM(double sensorUnits) {
    return (sensorUnits / 2048.0) * 600.0;
  }
  
  public double RPMtoFalconUnits(double RPM) {
    return (RPM / 600.0) * 2048.0;
  }

  public double getLeftRPM(){
    return falconUnitsToRPM(leftFlyWheel.getSelectedSensorVelocity());
  }

  public double getRightRPM(){
    return falconUnitsToRPM(rightFlyWheel.getSelectedSensorVelocity());
  }

  public double getAverageRPM(){
    return (getLeftRPM() + getRightRPM()) / 2;
  }

  public double getTargetVelo(){
    return desiredVelo;
  }

  public boolean isOnTarget(){
    double rpmDifference = getTargetVelo() - getAverageRPM();

    if(rpmDifference <= ShooterConstants.shooterTreshold){
      return onTarget = true;
    } else {
      return onTarget = false;
    }
  }

  public void leftSetpoint(double setPoint){
    leftFlyWheel.set(TalonFXControlMode.Velocity, setPoint);
  }

  public void rightSetpoint(double setPoint){
    rightFlyWheel.set(TalonFXControlMode.Velocity, setPoint);
  }

  public void goToDashboardVelocity(){
    leftSetpoint(desiredVelo);
    rightSetpoint(desiredVelo);
  }

  public void coneDashboardVelo(){
    leftSetpoint(desiredVelo);
    rightSetpoint(desiredVelo);
  }

  public void setMotorsPower(double leftPower, double rightPower){
    leftFlyWheel.set(TalonFXControlMode.PercentOutput, leftPower);
    rightFlyWheel.set(TalonFXControlMode.PercentOutput, rightPower);
  }
}


