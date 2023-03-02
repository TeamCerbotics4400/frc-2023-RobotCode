// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDTunerCommand extends CommandBase {
  /** Creates a new PIDTunerCommand. */
  DriveTrain m_drive;

  PIDController leftDriveController;
  PIDController rightDriveController; 

  double setPoint = 2.0;

  double tolerance = 0.005;

  public PIDTunerCommand(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;

    this.leftDriveController = m_drive.getLeftController();
    this.rightDriveController = m_drive.getRightController();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();

    leftDriveController.setTolerance(tolerance);
    rightDriveController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftDriveController.setSetpoint(setPoint);
    rightDriveController.setSetpoint(setPoint);

    double leftOutput = leftDriveController.calculate(m_drive.getLeftDistance());
    double rightOutput = rightDriveController.calculate(m_drive.getRightDistance());

    m_drive.straightDrive(leftOutput, rightOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftDriveController.atSetpoint() && rightDriveController.atSetpoint();
  }
}
