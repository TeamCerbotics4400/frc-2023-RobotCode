// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

//Relacion de 210:1
//Moverse a 90 grados cada lado
//Hall Effect DIO 1
public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  CANSparkMax wristMotor = new CANSparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);

  RelativeEncoder wristEncoder = wristMotor.getEncoder();

  SparkMaxPIDController wristController = wristMotor.getPIDController();

  DigitalInput hallEffectSensor = new DigitalInput(1);

  int smartMotionSlot = 0;

  public WristSubsystem() {
    wristMotor.restoreFactoryDefaults();

    wristMotor.setInverted(false);

    wristMotor.setIdleMode(IdleMode.kBrake);

    wristController.setP(WristConstants.kD);
    wristController.setI(WristConstants.kI);
    wristController.setD(WristConstants.kD);
    wristController.setFF(WristConstants.kFF);

    wristController.setSmartMotionMaxVelocity(0, smartMotionSlot);
    wristController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    wristController.setSmartMotionMaxAccel(0, smartMotionSlot);
    wristController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoder(){
    wristEncoder.setPosition(0);
  }

  public void turnToPosition(double setPoint){
    wristController.setReference(setPoint, ControlType.kSmartMotion);
  }
}
