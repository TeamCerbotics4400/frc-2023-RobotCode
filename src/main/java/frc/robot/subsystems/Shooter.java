// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax leftFlywheel = new CANSparkMax(ShooterConstants.LEFT_FLYWHEEL_ID, 
  MotorType.kBrushless);
  private CANSparkMax rightFlywheel = new CANSparkMax(ShooterConstants.RIGHT_FLYWHEEL_ID,
  MotorType.kBrushless);

  private Servo leftServo = new Servo(5);
  private Servo rightServo = new Servo(4);

  private RelativeEncoder leftFlywheelEncoder = leftFlywheel.getEncoder();
  private RelativeEncoder rightFlywheelEncoder = rightFlywheel.getEncoder();

  private SparkMaxPIDController leftFlywheel_PIDController = leftFlywheel.getPIDController();
  private SparkMaxPIDController rightFlywheel_PIDController = rightFlywheel.getPIDController();

  double desiredVelo = 0;

  public Shooter() {
    leftFlywheel.restoreFactoryDefaults();
    rightFlywheel.restoreFactoryDefaults();

    leftFlywheel.setInverted(true);
    rightFlywheel.setInverted(true);

    leftFlywheel.setCANTimeout(10);
    rightFlywheel.setCANTimeout(10);

    leftFlywheel.setIdleMode(IdleMode.kCoast);
    rightFlywheel.setIdleMode(IdleMode.kCoast);

    /* 
    int smartMotionSlot = 0;
    leftFlywheel_PIDController.setSmartMotionMaxVelocity(ShooterConstants.maxVel, smartMotionSlot);
    leftFlywheel_PIDController.setSmartMotionMinOutputVelocity(ShooterConstants.minVel, smartMotionSlot);
    leftFlywheel_PIDController.setSmartMotionMaxAccel(ShooterConstants.maxAcc, smartMotionSlot);
    leftFlywheel_PIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.allowedErr, smartMotionSlot);
    rightFlywheel_PIDController.setSmartMotionMaxVelocity(ShooterConstants.maxVel, smartMotionSlot);
    rightFlywheel_PIDController.setSmartMotionMinOutputVelocity(ShooterConstants.minVel, smartMotionSlot);
    rightFlywheel_PIDController.setSmartMotionMaxAccel(ShooterConstants.maxAcc, smartMotionSlot);
    rightFlywheel_PIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.allowedErr, smartMotionSlot);*/

    leftFlywheel_PIDController.setP(ShooterConstants.kP);
    leftFlywheel_PIDController.setD(ShooterConstants.kD);
    leftFlywheel_PIDController.setFF(ShooterConstants.kFF);
    leftFlywheel_PIDController.setOutputRange(ShooterConstants.kMinOutput, 
    ShooterConstants.kMaxOutput);

    rightFlywheel_PIDController.setP(ShooterConstants.kP);
    rightFlywheel_PIDController.setD(ShooterConstants.kD);
    rightFlywheel_PIDController.setFF(ShooterConstants.kFF);
    rightFlywheel_PIDController.setOutputRange(ShooterConstants.kMinOutput, 
    ShooterConstants.kMaxOutput);

    SmartDashboard.putNumber("Target Velo", desiredVelo);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double targetVelo = SmartDashboard.getNumber("Target Velo", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((desiredVelo != targetVelo )) {desiredVelo = targetVelo;}

    SmartDashboard.putNumber("Left RPM", getLeftRPM());
    SmartDashboard.putNumber("Right RPM", getRightRPM());

  }
    
  public void setLowerFlyWheelVelo(double setPoint){
    leftFlywheel_PIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }
  public void setUpperFlyWheelVelo(double setPoint){
    rightFlywheel_PIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public double getLeftRPM(){
    return leftFlywheelEncoder.getVelocity();
  }

  public double getRightRPM(){
    return rightFlywheelEncoder.getVelocity();
  }

  public void goToDashboardVelocity(){
    setLowerFlyWheelVelo(desiredVelo);
    setUpperFlyWheelVelo(desiredVelo);
  }

  public void setMotorsPower(double UpperPower, double LowerPower){
    leftFlywheel.set(LowerPower);
    rightFlywheel.set(UpperPower);
  }

  public void setServosAngle(double leftAngle, double rightAngle){
    leftServo.setAngle(leftAngle);
    rightServo.setAngle(rightAngle);
  }
}