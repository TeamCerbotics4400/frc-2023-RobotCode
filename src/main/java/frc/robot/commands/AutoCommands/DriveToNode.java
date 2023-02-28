// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NodeSelector;
import team4400.Util.GeomUtil;

/*Dividir cancha en 4 cuadrantes: Izquierda y Derecha (Con relacion al centro) y Charging Station
 * pasada o no. Con esa informacion determinar cual es el punto intermiedo mas adecuado y asignarlo.
 * Despues de eso continuar con la posicion del nodo seleccionada
 */
public class DriveToNode extends CommandBase {
  /** Creates a new DriveToTargetTest. */
  DriveTrain m_drive;
  NodeSelector m_nodeSelector;

  Joystick joy;

  Rotation2d closestRotation = new Rotation2d();

  double convergenceFactor = -0.60;

  DoubleLogEntry intermediatePoseLog;

  private PIDController angularController = 
  new PIDController(DriveConstants.TkP, DriveConstants.TkI,DriveConstants.TkD);

  public DriveToNode(DriveTrain m_drive, NodeSelector m_nodeSelector, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.m_nodeSelector = m_nodeSelector;
    this.joy = joy;

    angularController.enableContinuousInput(-180, 180);

    addRequirements(m_drive, m_nodeSelector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularController.reset();

    double testTarget = Double.POSITIVE_INFINITY;
    double distance = 
    m_nodeSelector.getNodeToAlign().getTranslation().getDistance(m_drive.getEstimationTranslation());
    if(distance < testTarget){
      closestRotation = m_nodeSelector.getNodeToAlign().getRotation();
      testTarget = distance;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d intermediatePoint = determineIntermediatePoint(m_drive.getPose());
    Rotation2d targetRotation = GeomUtil.direction(intermediatePoint
    .minus(m_drive.getEstimationTranslation()));

    angularController.setSetpoint(targetRotation.getDegrees());
    double angularSpeed = angularController.calculate(m_drive.getEstimationRotation()
      .plus(Rotation2d.fromDegrees(0)).getDegrees());
    angularSpeed = MathUtil.clamp(angularSpeed, -0.5, 0.5);

    m_drive.drive(-joy.getRawAxis(1), angularSpeed);

    SmartDashboard.putNumberArray("Intermediate pose", 
        new double[] {intermediatePoint.getX(), intermediatePoint.getY(), 
        closestRotation.getDegrees()});

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isChargingSCleared(){
    if(m_drive.getPose().getX() <= FieldConstants.CHARGING_STATION_CLEARENCE){
      return true;
    } else{
      return false;
    }
  }

  public boolean isOnRight(){
    if(m_drive.getPose().getY() >= FieldConstants.GRID_CENTER){
      return true;
    } else {
      return false;
    }
  }

  public Translation2d determineIntermediatePoint(Pose2d robotPose){
    double intermediateDistance = 
        m_nodeSelector.getNodeToAlign().getTranslation().getDistance(m_drive.getEstimationTranslation());
    Pose2d intermediatePose = new Pose2d(m_nodeSelector.getNodeToAlign().getTranslation(), closestRotation)
        .transformBy(GeomUtil.transformFromTranslation(intermediateDistance * convergenceFactor, 0.0));
    return intermediatePose.getTranslation(); 
  }

  
    /*if(!isChargingSCleared() && isOnRight()){
      return new Translation2d(2.60, 4.57);
    } else if(isChargingSCleared() && isOnRight()){
      return new Translation2d(1.0, 2.0); //Placeholder numbers
    } else if(!isChargingSCleared() && !isOnRight()){
      return new Translation2d(2.60, 0.75);
    } else{
      return new Translation2d(1.0, 2.0); //Placehodler numbers
    }
  }*/
}
