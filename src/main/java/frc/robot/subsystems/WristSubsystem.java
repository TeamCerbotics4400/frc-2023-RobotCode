// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

//Relacion de 210:1
//Moverse a 90 grados cada lado
public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  CANSparkMax wristMotor = new CANSparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);

  SparkMaxAbsoluteEncoder.Type kAbsEncType = SparkMaxAbsoluteEncoder.Type.kDutyCycle;

  public WristSubsystem() {
    wristMotor.restoreFactoryDefaults();

    wristMotor.setInverted(false);

    wristMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
