// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  DriveTrain m_drive;
  PIDController turnPID;

  public AutoAlign(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.turnPID = m_drive.getAlignController();
    turnPID.setTolerance(1);
    turnPID.enableContinuousInput(-180, 180);

    addRequirements(m_drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.setSetpoint(179);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.drive(0, turnPID.calculate(m_drive.getPose().getRotation().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }
}
