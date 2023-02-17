// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class DefaultIndexer extends CommandBase {
  /** Creates a new TestShooter. */
  IndexerSubsystem m_indexer;

  public DefaultIndexer(IndexerSubsystem m_indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_indexer = m_indexer;

    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shooter.goToDashboardVelocity();
    m_indexer.setNeoVelo(2000, 2000);
    /*if(m_shooter.isOnTarget()){
      m_feeder.setFeederPose(Constants.LinkageConstants.feederShooting);
      m_feeder.setFeederPower(1.0);
    } else{
      m_feeder.setFeederPose(Constants.LinkageConstants.feederShooting);
      m_feeder.setFeederPower(0.0);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setMotorsPower(0, 0);
    m_indexer.setMotorsPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
