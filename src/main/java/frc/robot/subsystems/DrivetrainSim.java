// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimulationConstants;

//26 de largo
//26 de ancho
public class DrivetrainSim extends SubsystemBase {
  /** Creates a new DrivetrainSim. */
  WPI_TalonSRX leftLeader = new WPI_TalonSRX(2);
  WPI_TalonSRX rightLeader = new WPI_TalonSRX(3);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(4);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(5);

  TalonSRXSimCollection leftSim = leftLeader.getSimCollection();
  TalonSRXSimCollection rightSim = rightLeader.getSimCollection();

  private final MotorControllerGroup leftGroup = 
  new MotorControllerGroup(leftLeader, leftFollower);

  private final MotorControllerGroup rightGroup = 
  new MotorControllerGroup(rightLeader, rightFollower);

  PIDController leftPIDController = new PIDController(0, 0, 0);
  PIDController rightPIDController = new PIDController(0, 0, 0);

  private Encoder leftEncoder = new Encoder(0, 1, false);
  private Encoder rightEncoder = new Encoder(2, 3, true);

  private Pigeon2 pigeonGyro = new Pigeon2(13);

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftGroup, rightGroup);

  private final DifferentialDriveOdometry m_odometry;

  public DifferentialDrivetrainSim m_driveSim;

  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;

  private final BasePigeonSimCollection simGyro = pigeonGyro.getSimCollection();

  private final Field2d simField;
  
  public DrivetrainSim() {
  
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();

    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);

    leftEncoder.setDistancePerPulse(SimulationConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(SimulationConstants.kEncoderDistancePerPulse);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
     0, 0);

     if(RobotBase.isSimulation()){
      m_driveSim = new DifferentialDrivetrainSim(SimulationConstants.kDriveGearbox, 
      SimulationConstants.kDriveGearing, 15, 52, 
      SimulationConstants.kWheelDiameterMeters / 2, SimulationConstants.kTrackwidthMeters, 
      VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
      

     simField = new Field2d();
     leftEncoderSim = new EncoderSim(leftEncoder);
     rightEncoderSim = new EncoderSim(rightEncoder);
     SmartDashboard.putData("Field", simField);

     } else{

      simField = null;
     leftEncoderSim = null;
     rightEncoderSim = null;

     }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), 
    leftEncoder.getDistance(), rightEncoder.getDistance());

    simField.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(leftGroup.get() * RobotController.getBatteryVoltage(), 
    rightGroup.get() * RobotController.getBatteryVoltage());

    m_driveSim.update(0.020);

    leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());

    simGyro.setRawHeading(-m_driveSim.getHeading().getDegrees());
  }

  public double getCurrentDrawnAmps(){
    return m_driveSim.getCurrentDrawAmps();
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeed(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_driveSim.setPose(pose);
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), 
    leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }

  public void arcadeDrive(double speed, double turn){
    double left = speed - turn;
    double right = speed + turn;

    leftGroup.set(left);
    rightGroup.set(right);
  }

  public void trankDriveVolts(double leftVoltage, double rightVoltage){
    leftGroup.setVoltage(leftVoltage);
    rightGroup.setVoltage(rightVoltage);
    diffDrive.feed();
  }

  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getAverageEncoderDistance(){
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder(){
    return leftEncoder;
  }

  public Encoder getRightEncoder(){
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput){
    diffDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading(){
    
  }

  public double getHeading(){
    return Math.IEEEremainder(pigeonGyro.getYaw(), 360) * (true ? -1.0 : 1.0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    
    leftLeader.set(ControlMode.PercentOutput, leftVolts / leftLeader.getBusVoltage());
    //rightMaster.set(rightVolts / 12);
    rightLeader.set(ControlMode.PercentOutput, rightVolts / rightLeader.getBusVoltage());   
  }

  public RamseteController disabledRamsete = new RamseteController() {
    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
    double angularVelocityRefRadiansPerSecond) {
      return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
    }
  };
  
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    if (initPose) {
      new InstantCommand(() -> {resetOdometry(trajectory.getInitialPose());}); 
    }
    RamseteCommand rCommand = new RamseteCommand(
      trajectory,
      this::getPose,
      new RamseteController(SimulationConstants.kRamseteB, SimulationConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        SimulationConstants.kS,
        SimulationConstants.kV,
        SimulationConstants.kA),
        SimulationConstants.kDriveKinematics,
        this::getWheelSpeed,
        leftPIDController,
        rightPIDController,
        // RamseteCommand passes volts to the callback
        this::tankDriveVolts,
        this);
        return rCommand;
      }
}
