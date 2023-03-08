// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

public class ActivateLevelShooter extends CommandBase {
  /** Creates a new ActivateLevelShooter. */
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;
  NodeSelector m_nodeSelector;

  public ActivateLevelShooter(ArmSubsystem m_arm, WristSubsystem m_wrist, NodeSelector m_nodeSelector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    this.m_nodeSelector = m_nodeSelector;

    addRequirements(m_arm, m_wrist, m_nodeSelector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ShootLevelSelection(m_arm, m_wrist, m_nodeSelector);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
