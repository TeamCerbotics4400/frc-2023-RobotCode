// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSim extends SubsystemBase {
  /** Creates a new DriveTrain. */
  WPI_TalonSRX leftLeader = new WPI_TalonSRX(0);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(1);
  WPI_TalonSRX rightLeader = new WPI_TalonSRX(2);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(3);

  TalonSRXSimCollection leftSim = leftLeader.getSimCollection();
  TalonSRXSimCollection rightSim = rightLeader.getSimCollection();

  private final MotorControllerGroup m_leftGroup = 
  new MotorControllerGroup(leftLeader, leftFollower);

  private final MotorControllerGroup m_rightGroup =
  new MotorControllerGroup(rightLeader, rightFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private Encoder m_leftEncoder = new Encoder(0, 1, true);

  private Encoder m_rightEncoder = new Encoder(0, 1, false);

  private Pigeon2 m_gyro = new Pigeon2(0);

  private final DifferentialDriveOdometry m_odometry;

  private DifferentialDrivetrainSim m_drivetrainSim;

  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;

  private Field2d m_fieldSim;

  BasePigeonSimCollection m_gyroSim;

  public DrivetrainSim() {
    
    m_leftEncoder.setDistancePerPulse(0);
    m_rightEncoder.setDistancePerPulse(0);

    leftLeader.configFactoryDefault();
    rightLeader.configFactoryDefault();

    leftFollower.configFactoryDefault();
    rightFollower.configFactoryDefault();

    m_odometry = new 
    DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), 
    0, 0);

    if(RobotBase.isSimulation()){
      
    }

    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    m_gyroSim = m_gyro.getSimCollection();

    m_fieldSim = new Field2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()),
     m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    SmartDashboard.putData("Field", m_fieldSim);
    
  }

  @Override
  public void simulationPeriodic(){
    m_drivetrainSim.setInputs(m_leftGroup.get() * RobotController.getBatteryVoltage(), 
    -m_rightGroup.get() * RobotController.getBatteryVoltage());
    m_drivetrainSim.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());

    m_gyroSim.setRawHeading(-m_drivetrainSim.getHeading().getDegrees());
  }

  //Odometry
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getYaw(), 360);
  }

  public void resetOdometry(Pose2d pose){
    m_drivetrainSim.setPose(pose);
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), 
    0, 0, pose);

  }

  public void resetEncoders(){
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void zeroHeading(){
    
  }

  public Encoder getLeftEncoder(){
    return m_leftEncoder;
  }

  public Encoder getRightEncoder(){
    return m_rightEncoder;
  }

  public double getAverangeEncoderDistance(){
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public void setMaxOutput(double maxOutput){
    m_drive.setMaxOutput(maxOutput);
  }

  public double getDrawnCurrentAmps(){
    return m_drivetrainSim.getCurrentDrawAmps();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void setTeleopControl(double speed, double turn){
    double left = speed - turn;
    double right = speed + turn;

    leftLeader.set(left);
    rightLeader.set(right);
  }



}
