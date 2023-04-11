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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FalconShooter extends SubsystemBase {
  /** Creates a new FalconShooter. */
  TalonFX leftFlyWheel = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_ID);
  TalonFX rightFlyWheel = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_ID);
  CANSparkMax horizontalFlyWheel = new CANSparkMax(ShooterConstants.HORIZONTAL_FLYWHEEL_ID,
                                                                      MotorType.kBrushless);

  RelativeEncoder neoEncoder = horizontalFlyWheel.getEncoder();

  SparkMaxPIDController neoController = horizontalFlyWheel.getPIDController();

  boolean onTarget = false;

  boolean intakeCone = false;

  double desiredVelo = 0;

  int pidSlot = 0;

  double stopCurrent = 7;

  //Beam DIO 0
  //Beam 2 DIO 3
  public FalconShooter() {
    leftFlyWheel.configFactoryDefault();
    rightFlyWheel.configFactoryDefault();

    leftFlyWheel.setInverted(true);
    rightFlyWheel.setInverted(false);

    leftFlyWheel.setNeutralMode(NeutralMode.Brake);
    rightFlyWheel.setNeutralMode(NeutralMode.Brake);

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

    neoController.setP(ShooterConstants.kP);
    neoController.setI(ShooterConstants.kI);
    neoController.setD(ShooterConstants.kD);
    neoController.setFF(ShooterConstants.kFF);

    SmartDashboard.putNumber("Desired velo", desiredVelo);

    SmartDashboard.putNumber("Shooter P", ShooterConstants.kP);
    SmartDashboard.putNumber("Shooter D", ShooterConstants.kP);
    SmartDashboard.putNumber("Shooter FF", ShooterConstants.kP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Left Current", leftFlyWheel.getStatorCurrent());
    //SmartDashboard.putNumber("Right Current", rightFlyWheel.getStatorCurrent());

    //SmartDashboard.putBoolean("NeedToStop", needToStop());

    SmartDashboard.putNumber("Left RPM", getLeftRPM());
    SmartDashboard.putNumber("Right RPM", getRightRPM());
    SmartDashboard.putNumber("Horizontal RPM", getHorizontalRPM());

    double targetVelo = SmartDashboard.getNumber("Desired Velo", 0);

    if(desiredVelo != targetVelo){desiredVelo = targetVelo;}

    double p = SmartDashboard.getNumber("Shooter P", ShooterConstants.kP);
    double d = SmartDashboard.getNumber("Shooter D", ShooterConstants.kD);
    double ff = SmartDashboard.getNumber("Shooter FF", ShooterConstants.kFF);

    if(ShooterConstants.kP != p){ShooterConstants.kP = p; 
                                 leftFlyWheel.config_kP(pidSlot, p); 
                                 rightFlyWheel.config_kP(pidSlot, p); 
                                 neoController.setP(p);}

    if(ShooterConstants.kD != d){ShooterConstants.kD = d; 
                                 leftFlyWheel.config_kD(pidSlot, d); 
                                 rightFlyWheel.config_kD(pidSlot, d); 
                                 neoController.setD(d);}

    if(ShooterConstants.kFF != ff){ShooterConstants.kFF = ff; 
                                 leftFlyWheel.config_kF(pidSlot, ff); 
                                 rightFlyWheel.config_kF(pidSlot, ff); 
                                 neoController.setFF(ff);}
    
  }

  public double getHorizontalRPM(){
    return neoEncoder.getVelocity();
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
    leftFlyWheel.set(TalonFXControlMode.Velocity, RPMtoFalconUnits(setPoint));
  }

  public void rightSetpoint(double setPoint){
    rightFlyWheel.set(TalonFXControlMode.Velocity, RPMtoFalconUnits(setPoint));
  }

  public void horizontalSetpoint(double setPoint){
    neoController.setReference(setPoint, ControlType.kVelocity);
  }

  public void goToDashboardVelocity(){
    leftSetpoint(desiredVelo);
    rightSetpoint(desiredVelo);
    horizontalSetpoint(desiredVelo);
  }

  public void setMotorsPower(double leftPower, double rightPower, double horizontalPower){
    leftFlyWheel.set(TalonFXControlMode.PercentOutput, leftPower);
    rightFlyWheel.set(TalonFXControlMode.PercentOutput, rightPower);
    horizontalFlyWheel.set(horizontalPower);
  }

  public boolean needToStop(){
    if(leftFlyWheel.getSupplyCurrent() > stopCurrent && rightFlyWheel.getSupplyCurrent() > stopCurrent){
      return true;
    } else {
      return false;
    }
  }

  public void stopShooterCurrent(){
    if(needToStop()){
      setMotorsPower(0, 0, 0);
    }
  }

  public double falconUnitsToRPM(double sensorUnits) {
    return (sensorUnits / 2048.0) * 600.0;
  }
  
  public double RPMtoFalconUnits(double RPM) {
    return (RPM / 600.0) * 2048.0;
  }
}


