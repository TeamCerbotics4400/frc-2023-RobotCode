// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconShooter;

public class IntakeCubes extends CommandBase {
  /** Creates a new DefaultIntake. */
  FalconShooter m_shooter;

  public IntakeCubes(FalconShooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_intake.setIntakePose(LinkageConstants.intakeExtended);
    
    m_shooter.leftSetpoint(-6000);
    m_shooter.rightSetpoint(-6000);
    m_shooter.horizontalSetpoint(-6000);

    m_shooter.stopShooterCurrent();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_intake.setIntakePose(0);
    m_shooter.setMotorsPower(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if(m_shooter.isShooterOcuppiedCube()){
      //return true;
      //} else{
        return false;
      //}
  }
}
