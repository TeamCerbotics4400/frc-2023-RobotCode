// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconShooter;

public class ShooterPID extends CommandBase {
  /** Creates a new ShooterPID. */
  FalconShooter m_shooter;
  public ShooterPID(FalconShooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.goToDashboardVelocity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorsPower(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
