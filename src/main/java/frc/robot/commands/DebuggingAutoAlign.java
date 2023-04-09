// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;

public class DebuggingAutoAlign extends CommandBase {
  /** Creates a new LimelightAutoAlign. */
  DriveTrain m_drive;

  PIDController angularController;
  //PIDController alignPID;

  public DebuggingAutoAlign(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;

    //alignPID = m_drive.getTurnPID();
    angularController = new PIDController(DriveConstants.TkP, DriveConstants.TkI, DriveConstants.TkD);

    angularController.enableContinuousInput(-180, 180);
    angularController.setTolerance(0.05);

    //alignPID.enableContinuousInput(-180, 180);
    //alignPID.setTolerance(0.05);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //alignPID.reset();
    angularController.reset();
    //LimelightHelpers.setLEDMode_ForceOn(VisionConstants.limelightName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angularController.setSetpoint(0);
    m_drive.drive(0, angularController.calculate(m_drive.getCorrectedAngle()));
    //m_drive.drive(-joy.getRawAxis(1), profiledAlignPID.calculate(m_drive.getCorrectedAngle()));
    //alignPID.setSetpoint(LimelightHelpers.getTY(VisionConstants.limelightName));
    //m_drive.drive(-joy.getRawAxis(1), alignPID.calculate(m_drive.getCorrectedAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.tapeLimelight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

