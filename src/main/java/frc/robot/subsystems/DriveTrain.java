// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.RobotPoseEstimator;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  CANSparkMax leftMaster = new CANSparkMax(DriveConstants.LeftMaster_ID, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(DriveConstants.LeftSlave_ID, MotorType.kBrushless);

  CANSparkMax rightMaster = new CANSparkMax(DriveConstants.RightMaster_ID, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(DriveConstants.RightSlave_ID, MotorType.kBrushless);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  //LimelightSubsystem limelight;

  Field2d field2d = new Field2d();

  MotorControllerGroup leftControllers = new MotorControllerGroup(leftMaster, leftSlave);
  MotorControllerGroup rightControllers = new MotorControllerGroup(rightMaster, rightSlave);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllers, rightControllers);

  Field2d m_field = new Field2d();

  RelativeEncoder encoderIzq = leftMaster.getEncoder();
  RelativeEncoder encoderDer = rightMaster.getEncoder();

  Pigeon2 imu = new Pigeon2(1);

  SparkMaxPIDController controladorIzq = leftMaster.getPIDController();
  SparkMaxPIDController controladorDer = rightMaster.getPIDController();

  Rotation2d rotacionChasis = new Rotation2d(getAngle());

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
  rotacionChasis,
  encoderCountsToMeters(encoderIzq.getPosition()), encoderCountsToMeters(encoderDer.getPosition()),
  new Pose2d(5.0, 13.5, new Rotation2d()));

  private final DifferentialDrivePoseEstimator m_poseEstimator =
            new DifferentialDrivePoseEstimator(
                    DriveConstants.kDriveKinematics, 
                    Rotation2d.fromDegrees(getAngle()), 
                    0.0, 0.0, new Pose2d());


  double kP = 0, kI = 0, kD = 0, kFF = 0, anguloObjetivo = 0;

  public PhotonCameraWrapper pcw;

  public DriveTrain(/*LimelightSubsystem limelightSubsystem*/) {

    //this.limelight = limelightSubsystem;

    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    leftMaster.setInverted(true);
    leftSlave.setInverted(true);

    SmartDashboard.putData("Field", m_field);
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    encoderIzq.setPosition(0);
    encoderDer.setPosition(0);

    controladorDer.setP(kP);
    controladorDer.setD(kD);
    controladorDer.setI(kI);
    controladorDer.setFF(0.5);

    imu.configFactoryDefault();

    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kD", 0);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kFF", 0);

    SmartDashboard.putNumber("Target Angle", 0);

    pcw = new PhotonCameraWrapper();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();

    odometry.update(Rotation2d.fromDegrees(getAngle()), 
    encoderCountsToMeters(encoderIzq.getPosition()), 
    encoderCountsToMeters(encoderDer.getPosition()));

    updateOdometryWVisionCorrection();

   
  }

  public void drive(double speed, double turn){
    differentialDrive.arcadeDrive(speed, turn);
  }

  public double getAngle(){
    return -imu.getPitch();
  }

  public double encoderCountsToMeters(double encoderCounts){
    double wheelRotations = encoderCounts / 10.75;
    double distance = wheelRotations * (Math.PI * 0.1524);
    return distance;
  }

  public double getDistance(){
    return (encoderCountsToMeters(encoderIzq.getPosition()) + 
          encoderCountsToMeters(encoderDer.getPosition())) / 2;
  }

  public void setDistance(double distancia){
  }

  public void resetSensors(){
    encoderDer.setPosition(0);
    encoderIzq.setPosition(0);
    imu.setYaw(0);
  }
  
  public void goToAngle(double setpoint){
    double target = 0.1051 * setpoint;
    controladorIzq.setReference(-target, ControlType.kPosition);
    controladorDer.setReference(target, ControlType.kPosition);
  }

  public double getTargetAngle(){
    return anguloObjetivo;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderCountsToMeters(encoderIzq.getPosition()),
    encoderCountsToMeters(encoderDer.getPosition()));
  }

  public void resetEncoders(){
    encoderIzq.setPosition(0);
    encoderDer.setPosition(0);
  }

  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(Rotation2d.fromDegrees(-getAngle()),
    encoderCountsToMeters(encoderIzq.getPosition()), encoderCountsToMeters(encoderDer.getPosition()),
      pose);
  }

  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllers.setVoltage(leftVolts);
    rightControllers.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  //Metodo de prueba
  public void updateOdometryWVisionCorrection(){
    m_poseEstimator.update(Rotation2d.fromDegrees(getAngle()), 
    encoderCountsToMeters(encoderIzq.getPosition()), 
    encoderCountsToMeters(encoderDer.getPosition()));

    Pair<Pose2d, Double> result = 
    pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    var cameraPose = result.getFirst();
    var camPoseObsTime = result.getSecond();

    if(cameraPose != null){
      m_poseEstimator.addVisionMeasurement(cameraPose, camPoseObsTime);
      m_field.getObject("Cam est Pose").setPose(cameraPose);
    } else {
      m_field.getObject("Cam est Pose").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }
  }

  
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    if (initPose) {
      new InstantCommand(() -> {resetOdometry(trajectory.getInitialPose());}); 
    }
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(2, 0.7),
            new SimpleMotorFeedforward(
                DriveConstants.kS,
                DriveConstants.kV,
                DriveConstants.kA),
                DriveConstants.kDriveKinematics,
            this::getWheelSpeeds,
            new PIDController(DriveConstants.kP, 0, 0),
            new PIDController(DriveConstants.kP, 0, 0),
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts,
            this);

    return ramseteCommand;

  }

  

  public void updateOdometry() {

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    }

    

  public void log(){
    SmartDashboard.putNumber("Distancia X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Distancia Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Angle", odometry.getPoseMeters().getRotation().getDegrees());

    //field2d.setRobotPose(limelight.getRobotPose().toPose2d());

    


    m_field.setRobotPose(odometry.getPoseMeters().getTranslation().getX(),
    odometry.getPoseMeters().getTranslation().getY(),
    odometry.getPoseMeters().getRotation());
    
    double p = SmartDashboard.getNumber("kP", 0);
    double i = SmartDashboard.getNumber("kI", 0);
    double d = SmartDashboard.getNumber("kD", 0);
    double ff = SmartDashboard.getNumber("kFF", 0);

    double targetAngle = SmartDashboard.getNumber("Target Angle", 0);

    if((p != kP)) { 
      controladorDer.setP(p); kP = p;
      controladorIzq.setP(p); kP = p;
     }
    if((i != kI)) { 
      controladorDer.setI(i); kI = i;
      controladorIzq.setI(i); kI = i;
     }
    if((d != kD)) { 
      controladorDer.setD(d); kD = d;
      controladorIzq.setD(d); kD = d;
     }
    if((ff != kFF)) { 
      controladorDer.setFF(ff); kFF = ff; 
      controladorIzq.setFF(ff); kFF = ff;
    }
    if(anguloObjetivo != targetAngle){
      anguloObjetivo = targetAngle;
    }
    
  }
}
