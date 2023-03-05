// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TestArm extends CommandBase {
  /** Creates a new TestArm. */
  ArmSubsystem m_arm;
  PIDController armController;

  public TestArm(ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.armController = m_arm.getArmController();

    armController.setTolerance(0.05);

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController.setSetpoint(160.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armController.calculate(m_arm.getEncoderAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setMotorsPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
