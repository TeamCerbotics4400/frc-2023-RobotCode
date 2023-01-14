// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class NeoIntake extends SubsystemBase {
  /** Creates a new NeoIntake. */

  CANSparkMax RPWheelIntake = new CANSparkMax(IntakeConstants.RapidWheeel_ID, MotorType.kBrushless);
  CANSparkMax I_Should_Be_A_Servo = new CANSparkMax(IntakeConstants.IShouldBeAServo_ID, MotorType.kBrushless);

  private SparkMaxPIDController RPWheelIntake_PIDController = RPWheelIntake.getPIDController();
  private SparkMaxPIDController ishouldbeaservo_PIDController = I_Should_Be_A_Servo.getPIDController();

  public NeoIntake() {
    RPWheelIntake.restoreFactoryDefaults();
    I_Should_Be_A_Servo.restoreFactoryDefaults();

    RPWheelIntake.setInverted(true);
    I_Should_Be_A_Servo.setInverted(true);

    RPWheelIntake.setCANTimeout(10);
    I_Should_Be_A_Servo.setCANTimeout(10);

    RPWheelIntake.setIdleMode(IdleMode.kCoast);
    I_Should_Be_A_Servo.setIdleMode(IdleMode.kCoast);
    int smartMotionSlot = 0;
    RPWheelIntake_PIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    RPWheelIntake_PIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    RPWheelIntake_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
