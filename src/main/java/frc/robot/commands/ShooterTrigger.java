// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeederLinkage;
import frc.robot.subsystems.FalconShooter;

public class ShooterTrigger extends CommandBase {
  /** Creates a new TestShooter. */
  FeederLinkage m_feeder;
  FalconShooter m_shooter;

  double fIdle = 0.0;
  double fTrigger = 0.0;

  public ShooterTrigger(FeederLinkage m_feeder, FalconShooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_feeder = m_feeder;
    this.m_shooter = m_shooter;

    addRequirements(m_shooter, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_shooter.goToDashboardVelocity();
    m_feeder.setFeederPose(Constants.LinkageConstants.feederShooting);
    m_feeder.setFeederPower(1.0);
    if(m_shooter.isOnTarget()){
      m_feeder.setFeederPose(Constants.LinkageConstants.feederShooting);
      m_feeder.setFeederPower(1.0);
    } else{
      m_feeder.setFeederPose(Constants.LinkageConstants.feederShooting);
      m_feeder.setFeederPower(0.0);
    }
  
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setMotorsPower(0, 0);
    m_feeder.setFeederPose(0);
    m_feeder.setFeederPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
