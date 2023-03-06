// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;

public class CubeShooter extends CommandBase {
  /** Creates a new DefaultShooter. */
  private FalconShooter shooter;
  //private ArmSubsystem m_arm;

  public CubeShooter(FalconShooter shooter){//ArmSubsystem m_arm) {
    this.shooter = shooter;
    //this.m_arm = m_arm;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.goToDashboardVelocity();
    /*if(m_arm.getMeasurement() < m_arm.getController().getGoal().position + ArmConstants.ARM_THRESHOLD && 
    m_arm.getMeasurement() > m_arm.getController().getGoal().position - ArmConstants.ARM_THRESHOLD){*/
    shooter.goToDashboardVelocity();
    /* } else {
      shooter.setMotorsPower(0, 0);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setMotorsPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
