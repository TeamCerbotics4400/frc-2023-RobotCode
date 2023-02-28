// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  CANSparkMax leftLeader = new CANSparkMax(DriveConstants.LeftLeader_ID, MotorType.kBrushless);
  CANSparkMax leftFollower = new CANSparkMax(DriveConstants.LeftFollower_ID, MotorType.kBrushless);

  CANSparkMax rightLeader = new CANSparkMax(DriveConstants.RightLeader_ID, MotorType.kBrushless);
  CANSparkMax rightFollower = new CANSparkMax(DriveConstants.RightFollower_ID, MotorType.kBrushless);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  private PIDController balancePID = new PIDController(0.029, 0, 0.0001); 

  private PIDController alignPID = new PIDController(DriveConstants.TkP, DriveConstants.TkI, DriveConstants.TkD);

  //LimelightSubsystem limelight;

  MotorControllerGroup leftControllers = new MotorControllerGroup(leftLeader, leftFollower);
  MotorControllerGroup rightControllers = new MotorControllerGroup(rightLeader, rightFollower);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllers, rightControllers);

  Field2d m_field = new Field2d();

  RelativeEncoder leftEncoder = leftLeader.getEncoder();
  RelativeEncoder rightEncoder = rightLeader.getEncoder();

  Pigeon2 imu = new Pigeon2(13);

  SparkMaxPIDController controladorIzq = leftLeader.getPIDController();
  SparkMaxPIDController controladorDer = rightLeader.getPIDController();

  //private Pose2d mPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(getAngle()));

  private final DifferentialDrivePoseEstimator m_poseEstimator =
            new DifferentialDrivePoseEstimator(
                    DriveConstants.kDriveKinematics, 
                    Rotation2d.fromDegrees(getAngle()), 
                    0.0, 0.0, new Pose2d());
  
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
                      new Rotation2d(m_poseEstimator.getEstimatedPosition()
                                      .getRotation().getDegrees()), 
                      m_poseEstimator.getEstimatedPosition().getX(),
                      m_poseEstimator.getEstimatedPosition().getY());


  double kP = 0, kI = 0, kD = 0, kFF = 0.5, anguloObjetivo = 0;

  ShuffleboardTab debuggingTab;
  ShuffleboardTab competitionTab;

  public PhotonCameraWrapper pcw;

  public DriveTrain(/*LimelightSubsystem limelightSubsystem*/) {

    //this.limelight = limelightSubsystem;

    leftLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    rightLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    rightLeader.setInverted(true);
    rightFollower.setInverted(true);

    leftLeader.setInverted(false);
    leftFollower.setInverted(false);

    SmartDashboard.putData("Field", m_field);
    
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    controladorDer.setP(kP);
    controladorDer.setD(kD);
    controladorDer.setI(kI);
    controladorDer.setFF(kFF);

    imu.configFactoryDefault();
    
    SmartDashboard.putNumber("Target Angle", 0);

    debuggingTab = Shuffleboard.getTab("Debugging Tab");
    competitionTab = Shuffleboard.getTab("Competition Tab");

    pcw = new PhotonCameraWrapper();

    PortForwarder.add(5800, "photonvision.local", 5800);

    SmartDashboard.putNumber("Balance P", balancePID.getP());
    SmartDashboard.putNumber("Balance I", balancePID.getI());
    SmartDashboard.putNumber("Balance D", balancePID.getD());

    resetImu();
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.resetPosition(new Rotation2d(m_poseEstimator.getEstimatedPosition()
    .getRotation().getDegrees()), 
    encoderCountsToMeters(leftEncoder.getPosition()),
    encoderCountsToMeters(rightEncoder.getPosition()),
     m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Rotation", 
    odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("Estimated X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Estimated Y", m_poseEstimator.getEstimatedPosition().getY());

    SmartDashboard.putNumber("Estimated Angle",
         m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    SmartDashboard.putNumber("PID Error", balancePID.getPositionError());

    double bP = SmartDashboard.getNumber("Balance P", alignPID.getP());
    double bI = SmartDashboard.getNumber("Balance I", alignPID.getI());
    double bD = SmartDashboard.getNumber("Balance D", alignPID.getD());

    if((bP != DriveConstants.TkP)){balancePID.setP(bP);}
    if((bI != DriveConstants.TkI)){balancePID.setI(bI);}
    if((bD != DriveConstants.TkD)){balancePID.setD(bD);}

    get3dPose();
  
  }

  public void selectDashboardType(){
    if(DriverStation.isFMSAttached()){
      Shuffleboard.getTab("Competition Tab");
    }
    else{
      Shuffleboard.getTab("Debugging Tab");
    }
  }

  public void drive(double speed, double turn){
    differentialDrive.arcadeDrive(speed, turn);
    differentialDrive.feed();
  }

  public double getAngle(){
    return  imu.getYaw();
  }

  public void resetImu(){
    imu.setYaw(0);
  }

  public double encoderCountsToMeters(double encoderCounts){
    double wheelRotations = encoderCounts / 10.75;
    double distance = wheelRotations * (Math.PI * 0.1524);
    return distance;
  }

  public double getDistance(){
    return (encoderCountsToMeters(leftEncoder.getPosition()) + 
          encoderCountsToMeters(rightEncoder.getPosition())) / 2;
  }

  public void resetSensors(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
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

  public Pose3d get3dPose(){
    Pose2d robotPose2d = getPose();

    return new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, 
    new Rotation3d(imu.getRoll(), getPitch(), robotPose2d.getRotation().getDegrees()));
  }

  public double getPitch(){
    return imu.getPitch();
  }

  public TrapezoidProfile.Constraints getDriveConstraints(){
    double maxSpeed = 5.2;
    double maxAcc = 9.0;

    return new TrapezoidProfile.Constraints(maxSpeed, maxAcc);
  }

  public void resetPoseWVision(){
    odometry.resetPosition(new Rotation2d(m_poseEstimator.getEstimatedPosition()
    .getRotation().getDegrees()), 
    encoderCountsToMeters(leftEncoder.getPosition()),
    encoderCountsToMeters(rightEncoder.getPosition()),
     m_poseEstimator.getEstimatedPosition());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderCountsToMeters(leftEncoder.getPosition()),
    encoderCountsToMeters(rightEncoder.getPosition()));
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetImu();
    odometry.resetPosition(Rotation2d.fromDegrees(getAngle()),
    encoderCountsToMeters(leftEncoder.getPosition()), encoderCountsToMeters(rightEncoder.getPosition()),
      pose);
  }

  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllers.setVoltage(leftVolts);
    rightControllers.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public Translation2d getEstimationTranslation(){
    return m_poseEstimator.getEstimatedPosition().getTranslation();
  }

  public Rotation2d getEstimationRotation(){
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  //Metodo de prueba
  public void updateOdometryWVisionCorrectionPhoton(){
    m_poseEstimator.update(Rotation2d.fromDegrees(getAngle()), 
    encoderCountsToMeters(leftEncoder.getPosition()), 
    encoderCountsToMeters(rightEncoder.getPosition()));

    Optional<EstimatedRobotPose> result = 
    pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if(result.isPresent()){
      EstimatedRobotPose camPose = result.get();
      m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), 
      camPose.timestampSeconds);
      m_field.getObject("Cam est Pose").setPose(camPose.estimatedPose.toPose2d());
    } else {
      m_field.getObject("Cam est Pose").setPose(m_poseEstimator.getEstimatedPosition());
    }

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

  }

  public void getEstimatedPose(){
    m_poseEstimator.getEstimatedPosition();
  }

  public PIDController getBalanceController(){
    return balancePID;
  }

  public PIDController getTurnPID(){
    return alignPID;
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
            leftPIDController,
            rightPIDController,
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts,
            this);

    return ramseteCommand;

  }
}
