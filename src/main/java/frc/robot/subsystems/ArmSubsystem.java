// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  CANSparkMax leftArm = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
  CANSparkMax rightArm = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);

  //Checar en rev client
  AbsoluteEncoder leftAbsoluteEncoder = leftArm.getAbsoluteEncoder(Type.kDutyCycle);
  //AbsoluteEncoder rightAbsoluteEncoder = rightArm.getAbsoluteEncoder(Type.kDutyCycle);

  SparkMaxPIDController leftArmController = leftArm.getPIDController();
  SparkMaxPIDController rightArmController = rightArm.getPIDController();

  private int smartMotionSlot = 0;

  public ArmSubsystem() {
    leftArm.restoreFactoryDefaults();
    rightArm.restoreFactoryDefaults();

    leftArm.setInverted(false);
    rightArm.setInverted(true);

    leftArm.setIdleMode(IdleMode.kBrake);
    rightArm.setIdleMode(IdleMode.kBrake);

    leftArmController.setP(ArmConstants.lkP);
    leftArmController.setI(ArmConstants.lkI);
    leftArmController.setD(ArmConstants.lkD);
    leftArmController.setFF(ArmConstants.lkFF);

    rightArmController.setP(ArmConstants.rkP);
    rightArmController.setI(ArmConstants.rkI);
    rightArmController.setD(ArmConstants.rkD);
    rightArmController.setFF(ArmConstants.rkFF);

    leftArmController.setSmartMotionMaxVelocity(ArmConstants.lmaxVel, smartMotionSlot);
    leftArmController.setSmartMotionMinOutputVelocity(ArmConstants.lkMinOutput, smartMotionSlot);
    leftArmController.setSmartMotionMaxAccel(ArmConstants.lmaxAcc, smartMotionSlot);
    leftArmController.setSmartMotionAllowedClosedLoopError(ArmConstants.lAllowedError, smartMotionSlot);

    rightArmController.setSmartMotionMaxVelocity(ArmConstants.rmaxVel, smartMotionSlot);
    rightArmController.setSmartMotionMinOutputVelocity(ArmConstants.rkMinOutput, smartMotionSlot);
    rightArmController.setSmartMotionMaxAccel(ArmConstants.rmaxAcc, smartMotionSlot);
    rightArmController.setSmartMotionAllowedClosedLoopError(ArmConstants.rAllowedError, smartMotionSlot);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Arm Encoder", leftAbsoluteEncoder.getPosition());
    //SmartDashboard.putNumber("Right Arm Encoder", rightAbsoluteEncoder.getPosition());
  }

  /*public void resetEncoder(){
    leftAbsoluteEncoder.
    rightAbsoluteEncoder.setPosition(0);
  }*/

  public void armToPosition(double setPoint){
    leftArmController.setReference(setPoint, ControlType.kSmartMotion);
    rightArmController.setReference(setPoint, ControlType.kSmartMotion);
  }
}
