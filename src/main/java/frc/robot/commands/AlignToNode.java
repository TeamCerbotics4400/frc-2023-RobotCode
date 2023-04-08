// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;

public class AlignToNode extends CommandBase {
  /** Creates a new AlignToNode. */
  DriveTrain m_drive;
  Joystick joy;

  private ArrayList<String> cubeNodes;

  PIDController angularController;

  public AlignToNode(DriveTrain m_drive, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.joy = joy;

    cubeNodes = new ArrayList<String>();

    cubeNodes.add("Node 2 Cube");
    cubeNodes.add("Node 5 Cube");
    cubeNodes.add("Node 8 Cube");


    angularController = new PIDController(DriveConstants.TkP, DriveConstants.TkI, DriveConstants.TkD);

    angularController.enableContinuousInput(-180, 180);
    angularController.setTolerance(0.05);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularController.reset();
    //angularController.setSetpoint(-LimelightHelpers.getTX(VisionConstants.tagLimelightName));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if(cubeNodes.contains(getNearestNode())){
      profiledAlignPID.setGoal(-LimelightHelpers.getTX(VisionConstants.tagLimelightName));
    } else {
      profiledAlignPID.setGoal(LimelightHelpers.getTY(VisionConstants.tapeLimelight));
      LimelightHelpers.setLEDMode_ForceOn(VisionConstants.tapeLimelight);
    }*/
    
    
    m_drive.drive(-joy.getRawAxis(1), angularController.calculate(LimelightHelpers.getTX(VisionConstants.tagLimelightName)));

    SmartDashboard.putString("Nearest Node", getNearestNode());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.tapeLimelight);
  }

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
