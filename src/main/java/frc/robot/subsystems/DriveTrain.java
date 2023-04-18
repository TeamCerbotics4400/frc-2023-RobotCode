// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import team4400.StateMachines;
import team4400.Util.DriveSignal;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  CANSparkMax leftLeader = new CANSparkMax(DriveConstants.LeftLeader_ID, MotorType.kBrushless);
  CANSparkMax leftFollower = new CANSparkMax(DriveConstants.LeftFollower_ID, MotorType.kBrushless);

  CANSparkMax rightLeader = new CANSparkMax(DriveConstants.RightLeader_ID, MotorType.kBrushless);
  CANSparkMax rightFollower = new CANSparkMax(DriveConstants.RightFollower_ID, MotorType.kBrushless);

  MotorControllerGroup leftControllers = new MotorControllerGroup(leftLeader, leftFollower);
  MotorControllerGroup rightControllers = new MotorControllerGroup(rightLeader, rightFollower);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  private PIDController balancePID = new PIDController(-0.024, 0, -0.0016); 

  private PIDController alignPID = new PIDController(DriveConstants.TkP, DriveConstants.TkI, DriveConstants.TkD);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllers, rightControllers);

  RelativeEncoder leftEncoder = leftLeader.getEncoder();
  RelativeEncoder rightEncoder = rightLeader.getEncoder();

  Pigeon2 imu = new Pigeon2(DriveConstants.Gyro_ID);

  SparkMaxPIDController controladorIzq = leftLeader.getPIDController();
  SparkMaxPIDController controladorDer = rightLeader.getPIDController();
  
  /* 
   * We decided that having two types of odometry would work better than just having one of them.
   * The visionOdometry is used for the TeleOp period, it helps us find our way on the field
   * using vision correction (hence the name *VISION*Odometry)
   * 
   * On the other hand the wheelOdometry class helps us use the classical encoder getPosition() to
   * get measurements and avoid having lots of noise on our measurements for more accurate path
   * following 
   */

  DifferentialDriveOdometry wheelOdometry = new DifferentialDriveOdometry(
    Rotation2d.fromDegrees(getCorrectedAngle()), encoderCountsToMeters(leftEncoder.getPosition()), 
    encoderCountsToMeters(rightEncoder.getPosition()));
  
  //ShuffleboardTab debuggingTab;
  //ShuffleboardTab competitionTab;

  private VisionSystem vision = new VisionSystem(this);

  private DoubleArrayLogEntry bot3dPose;

  //Relacion: 8.41 : 1
  //Diametro de llantas: 6 in
  //Units per Rotation : 0.4788
  //Ancho con bumpers = 29.5 in/0.7493 m
  //Largo con bumpers = 33.5 in/0.8509 m 
  public DriveTrain() {

    leftLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    rightLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeader.clearFaults();
    leftFollower.clearFaults();

    rightLeader.clearFaults();
    rightFollower.clearFaults();

    rightLeader.setInverted(false);
    rightFollower.setInverted(false);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setSmartCurrentLimit(40);//75);
    rightLeader.setSmartCurrentLimit(40);//75);
    leftFollower.setSmartCurrentLimit(40);//75);
    rightFollower.setSmartCurrentLimit(40);//75);

    imu.configFactoryDefault();

    //debuggingTab = Shuffleboard.getTab("Debugging Tab");
    //competitionTab = Shuffleboard.getTab("Competition Tab");

    resetImu();
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     wheelOdometry.update(Rotation2d.fromDegrees(getCorrectedAngle()), 
     encoderCountsToMeters(leftEncoder.getPosition()), 
     encoderCountsToMeters(rightEncoder.getPosition()));

    SmartDashboard.putString("Position State", StateMachines.getPositionState().toString());

    

    //SmartDashboard.putNumber("Wheel X", wheelOdometry.getPoseMeters().getX());
    //SmartDashboard.putNumber("Wheel Y", wheelOdometry.getPoseMeters().getY());
    //SmartDashboard.putNumber("Wheel Angle", wheelOdometry.getPoseMeters().getRotation().getDegrees());

    //bot3dPose.append(log3dPose());
  }

  /*public void selectDashboardType(){
    if(DriverStation.isFMSAttached()){
      Shuffleboard.getTab("Competition Tab");
    }
    else{
      Shuffleboard.getTab("Debugging Tab");
    }
  }*/

  public void setDriveCurrentLimit(int current){
    leftLeader.setSmartCurrentLimit(current);
    rightLeader.setSmartCurrentLimit(current);
    leftFollower.setSmartCurrentLimit(current);
    rightFollower.setSmartCurrentLimit(current);
  }

  /*********** Drive Methods ***********/

  //Normal arcadeDrive method for miscellaneous things
  public void drive(double speed, double turn){
    differentialDrive.arcadeDrive(speed, turn);
  }

  /* 
   * These methods below are for the Cheesyish Drive method, a smoother and more precise way
   * to drive the robot, depending on the axis of the robot it calculates a smoother turn
   * with some parameters that are given. This is similar to the curvatureDrive method of the 
   * DifferentialDrive class but we are using this one because we feel like we have more control
   * of every parameter needed. 
   */
  public void setOpenLoop(DriveSignal signal){
    leftLeader.set(signal.getLeft());
    rightLeader.set(signal.getRight());
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }
  private static final double kEpsilon = 1E-9;//-------
  public static DriveSignal inverseKinematics(Twist2d velocity) {
    if (Math.abs(velocity.dtheta) < kEpsilon) {
      return new DriveSignal(velocity.dx, velocity.dx);
    }
    double delta_v = DriveConstants.TRACK_WIDTH_INCHES * velocity.dtheta / (2 * DriveConstants.TRACK_SCRUB_FACTOR);
    return new DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
  }
  
  public void setCheesyishDrive(Joystick joystick){
    setCheesyishDrive(0.8 * setJoyDeadBand(-joystick.getRawAxis(1), 0.15) ,0.8 * setJoyDeadBand(-joystick.getRawAxis(4), 0.15) , joystick.getRawButton(10));
  }
  
  public static double setJoyDeadBand(double joystickValue, double deadBand) {
    return joystickValue < deadBand && joystickValue > -deadBand ? 0 : joystickValue;
  }

  public void setCheesyishDrive(double throttle, double wheel, boolean quickTurn){
    
    if (epsilonEquals(throttle, 0.0, 0.075)) {
      throttle = 0.0;
    }
    
    if (epsilonEquals(wheel, 0.0, 0.075)) {
      wheel = 0.0;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }
       
    wheel *= kWheelGain;
    DriveSignal signal = inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    differentialDrive.feed();
  }

  /*********** Odometry ***********/

  public double getAngle(){
    return imu.getYaw();
  }

  public double getCorrectedAngle(){
    return Math.IEEEremainder(getAngle(), 360);
  }

  public void resetImu(){
    imu.setYaw(0);
  }

  public double getDistance(){
    return (encoderCountsToMeters(leftEncoder.getPosition()) + 
          encoderCountsToMeters(rightEncoder.getPosition())) / 2;
  }

  public void resetSensors(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public Pose2d getWheelPose(){
    return wheelOdometry.getPoseMeters();
  }

  public double getPitch(){
    return imu.getPitch();
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
    wheelOdometry.resetPosition(Rotation2d.fromDegrees(getAngle()),
    encoderCountsToMeters(leftEncoder.getPosition()), 
    encoderCountsToMeters(rightEncoder.getPosition()), pose);
  }

  public double getLeftDistance(){
    return encoderCountsToMeters(leftEncoder.getPosition());
  }

  public double getRightDistance(){
    return encoderCountsToMeters(rightEncoder.getPosition());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllers.setVoltage(leftVolts);
    rightControllers.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public Pose2d getVisionPose(){
    return vision.estimatedPose2d();
  }

  public void resetVisionPose(Pose2d pose){
    vision.resetPoseEstimator(pose);
  }

  public void setAllianceForVision(Alliance alliance){
    vision.setAlliance(alliance);
  }

  //Gets the Balance PID Controller for use in other classes
  public PIDController getBalanceController(){
    return balancePID;
  }

  //Gets the Balance PID Controller for use in other classes
  public PIDController getTurnPID(){
    return alignPID;
  }

  /* 
   * We do this to have the ability to send our current PID controller values to our Shuffleboard 
   * and using a tool called TunableNumber we can fine-tune our PID controllers without the need to be
   * deploying code after every change.
   */

   /*********** Autos ***********/

  //Ramsete Command for following created Trajectories
  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory) {
    PPRamseteCommand ramseteCommand =
        new PPRamseteCommand(
            trajectory,
            this::getWheelPose,
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
            false,
            this);

    return ramseteCommand;
  }

  public Command createCommandForTrajectoryVision(PathPlannerTrajectory trajectory) {
    PPRamseteCommand ramseteCommand =
        new PPRamseteCommand(
            trajectory,
            this::getVisionPose,
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
            false,
            this);

    return ramseteCommand;
  }

  /*********** Unit Conversions ***********/
  
  public double encoderCountsToMeters(double encoderCounts){
    double wheelRotations = encoderCounts / 8.41;
    double distance = wheelRotations * (Math.PI * 0.1524);
    return distance;
  }
}
