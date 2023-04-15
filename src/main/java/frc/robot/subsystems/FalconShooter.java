// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import team4400.Util.Interpolation.InterpolatingDouble;
import team4400.Util.Interpolation.InterpolatingTreeMap;

public class FalconShooter extends SubsystemBase {
  /** Creates a new FalconShooter. */
  TalonFX leftFlyWheel = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_ID);
  TalonFX rightFlyWheel = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_ID);
  CANSparkMax horizontalFlyWheel = new CANSparkMax(ShooterConstants.HORIZONTAL_FLYWHEEL_ID,
                                                                      MotorType.kBrushless);

  RelativeEncoder neoEncoder = horizontalFlyWheel.getEncoder();
  SparkMaxPIDController neoController = horizontalFlyWheel.getPIDController();

  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
    kDistanceToShooterHighSpeed = new InterpolatingTreeMap<>();

    static{
      kDistanceToShooterHighSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
    }

  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
   kDistanceToShooterMidSpeed = new InterpolatingTreeMap<>();

    static{
      kDistanceToShooterMidSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
    }

  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      kDistanceToShooterLowSpeed = new InterpolatingTreeMap<>();

    static{
      kDistanceToShooterLowSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
    }

  boolean onTarget = false;

  double falconDesiredVelo = 0;
  double neoDesiredVelo = 0;

  int pidSlot = 0;

  double stopCurrent = 13;

  double shootCurrent = 7.0;

  //Reduccion Falcon = 3/1
  //Reduccion Neo = 2/1
  public FalconShooter() {

    leftFlyWheel.configFactoryDefault();
    rightFlyWheel.configFactoryDefault();
    horizontalFlyWheel.restoreFactoryDefaults();

    leftFlyWheel.setInverted(true);
    rightFlyWheel.setInverted(false);
    horizontalFlyWheel.setInverted(true);

    leftFlyWheel.setNeutralMode(NeutralMode.Brake);
    rightFlyWheel.setNeutralMode(NeutralMode.Brake);
    horizontalFlyWheel.setIdleMode(IdleMode.kCoast);

    leftFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 5, 0.5));
    rightFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 5, 0.5));

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

    neoController.setP(ShooterConstants.hKp);
    neoController.setD(ShooterConstants.hKd);
    neoController.setFF(ShooterConstants.hKff);

    SmartDashboard.putNumber("Falcon velo", falconDesiredVelo);
    SmartDashboard.putNumber("Neo velo", neoDesiredVelo);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("Left Current Filtered", filter.calculate(leftFlyWheel.getStatorCurrent()));
    //SmartDashboard.putNumber("Right Current Filtered", filter.calculate(rightFlyWheel.getStatorCurrent()));

    //SmartDashboard.putNumber("Horizontal Roller Current", horizontalFlyWheel.getOutputCurrent());

    SmartDashboard.putBoolean("Has already Shot", hasAlreadyShot());

    SmartDashboard.putNumber("Horizontal velo", neoEncoder.getVelocity());
    SmartDashboard.putNumber("Left Velo", getLeftRPM());
    SmartDashboard.putNumber("Right Velo", getRightRPM());

    double falconVelo = SmartDashboard.getNumber("Falcon velo", 0);
    double neoVelo = SmartDashboard.getNumber("Neo velo", 0);

    if(falconDesiredVelo != falconVelo){falconDesiredVelo = falconVelo;}
    if(neoDesiredVelo != neoVelo){neoDesiredVelo = neoVelo;}

    SmartDashboard.putBoolean("NeedToStop", needToStop());
  }

  public void setCurrentLimit(double current, double seconds){
    leftFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, current, 5, seconds));
    rightFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, current, 5, seconds));
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
    return falconDesiredVelo;
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
    leftSetpoint(falconDesiredVelo);
    rightSetpoint(falconDesiredVelo);
    horizontalSetpoint(neoDesiredVelo);
  }

  public void setMotorsPower(double leftPower, double rightPower, double horizontalPower){
    leftFlyWheel.set(TalonFXControlMode.PercentOutput, leftPower);
    rightFlyWheel.set(TalonFXControlMode.PercentOutput, rightPower);
    horizontalFlyWheel.set(horizontalPower);
  }

  public boolean needToStop(){
    if(filter.calculate(leftFlyWheel.getSupplyCurrent()) > stopCurrent && filter.calculate(rightFlyWheel.getSupplyCurrent()) > stopCurrent){
      return true;
    } else {
      return false;
    }
  }

  public boolean hasAlreadyShot(){
    if(filter.calculate(leftFlyWheel.getSupplyCurrent()) > shootCurrent && filter.calculate(rightFlyWheel.getSupplyCurrent()) > shootCurrent){
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


