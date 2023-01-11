// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  CANSparkMax leftMaster = new CANSparkMax(DriveConstants.LeftMaster_ID, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(DriveConstants.LeftSlave_ID, MotorType.kBrushless);

  CANSparkMax rightMaster = new CANSparkMax(DriveConstants.RightMaster_ID, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(DriveConstants.RightSlave_ID, MotorType.kBrushless);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
