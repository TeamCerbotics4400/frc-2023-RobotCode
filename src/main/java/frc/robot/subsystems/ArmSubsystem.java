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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);

  RelativeEncoder leftArmEncoder = leftArmMotor.getEncoder();
  RelativeEncoder rightArmEncoder = rightArmMotor.getEncoder();

  SparkMaxPIDController leftArmController = leftArmMotor.getPIDController();
  SparkMaxPIDController rightArmController = rightArmMotor.getPIDController();

  public ArmSubsystem() {
    leftArmMotor.restoreFactoryDefaults();

    leftArmMotor.setInverted(false);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Arm Encoder", leftArmEncoder.getPosition());
    SmartDashboard.putNumber("Right Arm Encoder", rightArmEncoder.getPosition());
  }

  public void resetEncoder(){
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);
  }

  public void armGoToPosition(double setPoint){
    leftArmController.setReference(setPoint, ControlType.kSmartMotion);
    rightArmController.setReference(setPoint, ControlType.kSmartMotion);
  }
}
