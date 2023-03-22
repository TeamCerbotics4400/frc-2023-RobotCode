// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TeleOpControl extends CommandBase {
  /** Creates a new TeleOpControl. */
  DriveTrain m_drive;
  Joystick joy;
  public TeleOpControl(DriveTrain m_drive, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.joy = joy;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setCheesyishDrive(joy);

    if(joy.getRawAxis(2) > 0.15){
      m_drive.setCheesyishDrive(0.0, joy.getRawAxis(2)
      , true);
    }

    if(joy.getRawAxis(3) > 0.15){
      m_drive.setCheesyishDrive(0.0, -joy.getRawAxis(3), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
