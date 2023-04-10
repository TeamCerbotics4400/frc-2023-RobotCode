// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FalconShooter extends SubsystemBase {
  /** Creates a new FalconShooter. */
  TalonFX leftFlyWheel = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_ID);
  TalonFX rightFlyWheel = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_ID);

  DigitalInput beamSensor = new DigitalInput(0);
  DigitalInput beamSensor2 = new DigitalInput(3);

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("Average RPM", getAverageRPM());
    SmartDashboard.putNumber("Left Current", leftFlyWheel.getStatorCurrent());
    SmartDashboard.putNumber("Right Current", rightFlyWheel.getStatorCurrent());

    SmartDashboard.putBoolean("NeedToStop", needToStop());
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
    leftFlyWheel.set(TalonFXControlMode.Velocity, RPMtoFalconUnits(setPoint));
  }

  public void rightSetpoint(double setPoint){
    rightFlyWheel.set(TalonFXControlMode.Velocity, RPMtoFalconUnits(setPoint));
  }

  public void goToDashboardVelocity(){
    leftSetpoint(desiredVelo);
    rightSetpoint(desiredVelo);
  }

  public void setMotorsPower(double leftPower, double rightPower){
    leftFlyWheel.set(TalonFXControlMode.PercentOutput, leftPower);
    rightFlyWheel.set(TalonFXControlMode.PercentOutput, rightPower);
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
      setMotorsPower(0, 0);
    }
  }

  public void stopShooterSensorCube(){
    if(isShooterOcuppiedCube()){
      setMotorsPower(0, 0);
    }
  }

  public void stopShooterSensorCone(){
    if(isShooterOcuppiedCone()){
      setMotorsPower(0, 0);
    }
  }

  public boolean isShooterOcuppiedCube(){
    if(beamSensor.get() != true){
      return true;
    } else{
      return false;
    }
  }

  public boolean isShooterOcuppiedCone(){
    if(beamSensor2.get() != true){
      return true;
    } else{
      return false;
    }
  }
}


