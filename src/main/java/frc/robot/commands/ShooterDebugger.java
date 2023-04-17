// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import team4400.StateMachines;

public class ShooterDebugger extends CommandBase {
  /** Creates a new ShooterPID. */
  FalconShooter m_shooter;
  NodeSelector m_selector;
  public ShooterDebugger(FalconShooter m_shooter, NodeSelector m_selector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_selector = m_selector;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setCurrentLimit(80, 1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shooter.goToDashboardVelocity();
    m_shooter.leftSetpoint(m_shooter.getSpeedForDistanceFalconHigh(
      LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelightName).getZ()));
    m_shooter.rightSetpoint(m_shooter.getSpeedForDistanceFalconHigh(
      LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelightName).getZ()));
    m_shooter.horizontalSetpoint(m_shooter.getSpeedForDistanceNeoHigh(
      LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelightName).getZ()));
    //SmartDashboard.putNumber("Distance to Target", getLimeDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorsPower(0, 0, 0);
    StateMachines.setIntakeIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double getLimeDistance(){
    double distance = 0;
    switch(m_selector.getLevelName().toString()){
      case "High":
      distance = (VisionConstants.HEIGHT_OF_HIGH_NODE - VisionConstants.LIMELIGHT_FLOOR_CLEREANCE) /
      Math.tan(Math.toRadians(LimelightHelpers.getTY(VisionConstants.tagLimelightName)));
      break;

      case "Mid":
      distance = (VisionConstants.HEIGHT_OF_MID_NODE - VisionConstants.LIMELIGHT_FLOOR_CLEREANCE) /
      Math.tan(Math.toRadians(LimelightHelpers.getTY(VisionConstants.tagLimelightName)));
      break;

      case "Low":
      distance = (VisionConstants.HEIGHT_OF_TAG - VisionConstants.LIMELIGHT_FLOOR_CLEREANCE) /
      Math.tan(Math.toRadians(LimelightHelpers.getTY(VisionConstants.tagLimelightName)));
      break;

      case "Ave Maria":
      distance = (VisionConstants.HEIGHT_OF_TAG - VisionConstants.LIMELIGHT_FLOOR_CLEREANCE) /
      Math.tan(Math.toRadians(LimelightHelpers.getTY(VisionConstants.tagLimelightName)));
      break;
    }
    return distance;
  }
}
