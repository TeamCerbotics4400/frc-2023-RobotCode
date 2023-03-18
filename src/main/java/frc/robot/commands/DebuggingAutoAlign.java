// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;

public class DebuggingAutoAlign extends CommandBase {
  /** Creates a new LimelightAutoAlign. */
  DriveTrain m_drive;

  Joystick joy;

  PIDController profiledAlignPID;
  //PIDController alignPID;

  public DebuggingAutoAlign(DriveTrain m_drive, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.joy = joy;

    //alignPID = m_drive.getTurnPID();
    profiledAlignPID = m_drive.getTurnPID();

    profiledAlignPID.enableContinuousInput(-180, 180);
    profiledAlignPID.setTolerance(0.05);

    //alignPID.enableContinuousInput(-180, 180);
    //alignPID.setTolerance(0.05);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //alignPID.reset();
    profiledAlignPID.reset();
    //LimelightHelpers.setLEDMode_ForceOn(VisionConstants.limelightName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    profiledAlignPID.setSetpoint(90);
    m_drive.setCheesyishDrive(joy.getRawAxis(1), profiledAlignPID.calculate(m_drive.getCorrectedAngle()), true);
    //m_drive.drive(-joy.getRawAxis(1), profiledAlignPID.calculate(m_drive.getCorrectedAngle()));
    //alignPID.setSetpoint(LimelightHelpers.getTY(VisionConstants.limelightName));
    //m_drive.drive(-joy.getRawAxis(1), alignPID.calculate(m_drive.getCorrectedAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.limelightName);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profiledAlignPID.atSetpoint();
  }
}

