// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class AlignToNode extends CommandBase {
  /** Creates a new AlignToNode. */
  DriveTrain m_drive;

  private PIDController angularController = 
  new PIDController(DriveConstants.TkP, DriveConstants.TkI,DriveConstants.TkD);

  public AlignToNode(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;

    angularController.enableContinuousInput(-180, 180);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angularController.setSetpoint(FieldConstants.BLUE_MAP.get(getNearestNode()).getRotation().getDegrees());

    m_drive.drive(0, angularController.calculate(m_drive.getEstimationRotation().getDegrees()));
    SmartDashboard.putString("Nearest Node", getNearestNode());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public String getNearestNode(){
    String nearestNode = null;
    double minDistance = Double.POSITIVE_INFINITY;

    for(Map.Entry<String, Pose2d> entry : FieldConstants.BLUE_MAP.entrySet()){
      double distance = entry.getValue().getTranslation().getDistance(m_drive.getEstimationTranslation());

      if(distance < minDistance){
        minDistance = distance;
        nearestNode = entry.getKey();
      }
    }

    return nearestNode;
  }
}
