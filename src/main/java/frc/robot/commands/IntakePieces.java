// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederLinkage;
import frc.robot.subsystems.IntakeLinkage;

public class IntakePieces extends CommandBase {
  /** Creates a new DefaultIntake. */
  IntakeLinkage m_intake;
  FeederLinkage m_feeder;

  double iDeployed = 0.0;
  double iRetracted = 0.0;

  double fIdle = 0.0;
  double fIntaking = 0.0;

  public IntakePieces(IntakeLinkage m_intake, FeederLinkage m_feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = m_intake;
    this.m_feeder = m_feeder;

    addRequirements(m_intake, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakePose(iDeployed);
    m_intake.setIntakePower(1.0);

    m_feeder.setFeederPose(fIntaking);
    m_feeder.setFeederPower(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePose(iRetracted);
    m_intake.setIntakePower(0.0);

    m_feeder.setFeederPose(fIdle);
    m_feeder.setFeederPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
