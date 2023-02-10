// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederLinkage;
import frc.robot.subsystems.IntakeLinkage;

public class IntakePieces extends CommandBase {
  /** Creates a new DefaultIntake. */
  IntakeLinkage m_Intake;
  FeederLinkage m_Feeder;

  double iDeployed = 0.0;
  double iRetracted = 0.0;

  double fIdle = 0.0;
  double fIntaking = 0.0;

  public IntakePieces(IntakeLinkage m_Intake, FeederLinkage m_Feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Intake = m_Intake;
    this.m_Feeder = m_Feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setIntakePose(iDeployed);
    m_Intake.setIntakePower(1.0);

    m_Feeder.setFeederPose(fIntaking);
    m_Feeder.setFeederPower(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setIntakePose(iRetracted);
    m_Intake.setIntakePower(0.0);

    m_Feeder.setFeederPose(fIdle);
    m_Feeder.setFeederPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
