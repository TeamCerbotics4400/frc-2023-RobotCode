// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import team4400.Util.GeomUtil;

/*Dividir cancha en 4 cuadrantes: Izquierda y Derecha (Con relacion al centro) y Charging Station
 * pasada o no. Con esa informacion determinar cual es el punto intermiedo mas adecuado y asignarlo.
 * Despues de eso continuar con la posicion del nodo seleccionada
 */
public class DriveToTargetTest extends CommandBase {
  /** Creates a new DriveToTargetTest. */
  DriveTrain m_drive;

  Joystick joy;

  Rotation2d closestRotation = new Rotation2d();

  double convergenceFactor = 0.75;

  private PIDController angularController = 
  new PIDController(DriveConstants.TkP, DriveConstants.TkI,DriveConstants.TkD);

  public DriveToTargetTest(DriveTrain m_drive, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.joy = joy;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularController.enableContinuousInput(-180, 180);
    angularController.reset();

    double testTarget = Double.POSITIVE_INFINITY;
    double distance = 
    FieldConstants.TEST_TAG.getTranslation().getDistance(m_drive.getEstimationTranslation());
    if(distance < testTarget){
      closestRotation = FieldConstants.TEST_TAG.getRotation();
      testTarget = distance;
    }

    Translation2d intermediatePose = determineIntermediatePoint(m_drive.getPose());

    Rotation2d targetRotation = GeomUtil.direction(intermediatePose
    .minus(m_drive.getEstimationTranslation()));

    angularController.setSetpoint(targetRotation.getDegrees());

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angularSpeed = angularController.calculate(m_drive.getEstimatioRotation()
    .plus(Rotation2d.fromDegrees(180)).getDegrees());

    angularSpeed = MathUtil.clamp(angularSpeed, -0.5, 0.5);

    m_drive.drive(-joy.getRawAxis(1), angularSpeed);
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
    double safeDistance = 
    FieldConstants.SAFE_TRANSLATION.getDistance(m_drive.getEstimationTranslation());

    //Pose2d intermediatePose = 
    //new Pose2d(FieldConstants.SAFE_TRANSLATION, closestRotation)
    //.transformBy(GeomUtil.transformFromTranslation(safeDistance * convergenceFactor, 0.0));

    return new Translation2d(3.00, 4.58);//intermediatePose.getTranslation();
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
