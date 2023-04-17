// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  private final DriveTrain m_drive;

  private double balancedAngle = 0.0;
  //private double balancedX = 3.85;
  
  public AutoBalance(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.getBalanceController().setSetpoint(balancedAngle);
    m_drive.getBalanceController().setIntegratorRange(-0.5, 0.5 );
    m_drive.getBalanceController().setTolerance(0.32);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_drive.getBalanceController().calculate(m_drive.getPitch()), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_drive.getBalanceController().atSetpoint() && isAtBalancedPose()){
      return true; 
    } else {
      return false;
    }
  }

  public boolean isAtBalancedPose(){
    if(m_drive.estimatedPose2d().getX() <= 3.98 &&  m_drive.estimatedPose2d().getX() >= 3.80){
      return true;
    } else {
      return false;
    }
  }
}
