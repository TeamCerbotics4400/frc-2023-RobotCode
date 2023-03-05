// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  CANSparkMax leftArm = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
  CANSparkMax rightArm = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);

  //SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  //Checar en rev client
  //RelativeEncoder alternateEncoder = leftArm.getAlternateEncoder(kAltEncType, 8192);

  PIDController armController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(2);
  //RelativeEncoder leftEncoder = leftArm.getEncoder();
  //RelativeEncoder rightEncoder = rightArm.getEncoder();
  //AbsoluteEncoder leftAbsoluteEncoder = leftArm.getAbsoluteEncoder(Type.kDutyCycle);
  //AbsoluteEncoder rightAbsoluteEncoder = rightArm.getAbsoluteEncoder(Type.kDutyCycle);

  /*ProfiledPIDController armController =  
  new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
   new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocityRadPerSecond, 
   ArmConstants.kMaxAccelerationMetersPerSecondSquared));*/

  SparkMaxPIDController leftController = leftArm.getPIDController();
  SparkMaxPIDController rightController = rightArm.getPIDController();

  //SparkMaxPIDController rightArmController = rightArm.getPIDController();

  ArmFeedforward armFeedforward;

  //private int smartMotionSlot = 0;

  
  private double setPoint;

  private double armPosition = 0.0;

  double targetAngle = 0.0;

  //Relacion 32.89 : 1

  public ArmSubsystem() {

    leftArm.restoreFactoryDefaults();
    rightArm.restoreFactoryDefaults();

    leftArm.setInverted(false);
    rightArm.follow(leftArm, true);

    leftArm.setIdleMode(IdleMode.kBrake);
    rightArm.setIdleMode(IdleMode.kBrake);

    armFeedforward = new ArmFeedforward(ArmConstants.kA, ArmConstants.kG, ArmConstants.kV);
    
    leftArm.setSoftLimit(SoftLimitDirection.kForward, softLimit(Rotation2d.fromDegrees(45)));
    leftArm.setSoftLimit(SoftLimitDirection.kReverse, softLimit(Rotation2d.fromDegrees(-45)));
    leftArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftArm.enableSoftLimit(SoftLimitDirection.kReverse, true);

    rightArm.setSoftLimit(SoftLimitDirection.kForward, softLimit(Rotation2d.fromDegrees(45)));
    rightArm.setSoftLimit(SoftLimitDirection.kReverse, softLimit(Rotation2d.fromDegrees(-45)));
    rightArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightArm.enableSoftLimit(SoftLimitDirection.kReverse, true);

    //160.5 90 grados real 
    //70.5 Diff
    //71.6 0 grados real
    // 71.6 Diff
    //252.5 180 grados real
    //72.5 Diff
    absoluteEncoder.setDistancePerRotation(360);
    
    SmartDashboard.putNumber("Desired arm angle", targetAngle);
    SmartDashboard.putNumber("Desired arm rotation", armPosition);

    SmartDashboard.putNumber("Arm P", ArmConstants.kP);
    SmartDashboard.putNumber("Arm I", ArmConstants.kI);
    SmartDashboard.putNumber("Arm D", ArmConstants.kD);
    //SmartDashboard.putNumber("Arm FF", ArmConstants.kFF);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Left Arm Encoder", leftAbsoluteEncoder.getPosition());
    //SmartDashboard.putNumber("Right Arm Encoder", rightAbsoluteEncoder.getPosition());

    //SmartDashboard.putNumber("Encoder angle", getAngle(alternateEncoder).getDegrees());
    SmartDashboard.putNumber("Raw Encoder", absoluteEncoder.getDistance());

    //SmartDashboard.putNumber("Controller setpoint", leftArmController.get);
    //SmartDashboard.putNumber("Right Encoder", getAngle(rightEncoder).getDegrees());

    double desiredPosition = SmartDashboard.getNumber("Desired arm angle", targetAngle);
    double desiredRotation = SmartDashboard.getNumber("Desired arm rotation", armPosition);

    if((desiredPosition != targetAngle)){desiredPosition = targetAngle;}
    if((desiredRotation != armPosition)){desiredRotation = armPosition;}

    double kP = SmartDashboard.getNumber("Arm P", ArmConstants.kP);
    double kI = SmartDashboard.getNumber("Arm I", ArmConstants.kI);
    double kD = SmartDashboard.getNumber("Arm D", ArmConstants.kD);
    //double kFF = SmartDashboard.getNumber("Arm FF", ArmConstants.kFF);

    if((kP != ArmConstants.kP)){armController.setP(kP); /*rightArmController.setP(kP);*/ 
                                  ArmConstants.kP = kP;}
    if((kI != ArmConstants.kI)){armController.setP(kI); /*rightArmController.setP(kI)*/; 
                                  ArmConstants.kI = kI;}
    if((kD != ArmConstants.kD)){armController.setP(kD); /*rightArmController.setP(kD)*/; 
                                  ArmConstants.kD = kD;}
    //if((kFF != ArmConstants.kFF)){leftArmController.setP(kFF); rightArmController.setP(kFF); 
                                  //ArmConstants.kP = kFF;}
  }

  public float softLimit(Rotation2d limit){
    return (float)(limit.getDegrees() / 180);
  }

  public synchronized Rotation2d getAngle(RelativeEncoder encoder) {
    return Rotation2d.fromDegrees(encoder.getPosition() * 180);
  }

  public void setArmAngle(double angle){
    leftController.setReference(angle, ControlType.kPosition, 
    0, armFeedforward.calculate(Math.toRadians(angle), 0));
  }

  public PIDController getArmController(){
    return armController;
  }

  public double getEncoderAngle(){
    return absoluteEncoder.getDistance();
  }

  /*public void armToPosition(double setPoint){
    leftArmController.setReference(setPoint, ControlType.kSmartMotion);
    //rightArmController.setReference(setPoint, ControlType.);
  }

  public void setAngle(Rotation2d angle){
    leftArmController.setReference(
      angle.getDegrees() / 180,
      ControlType.kSmartMotion);
      targetAngle = angle.getDegrees();
  }*/

  public void dashboardPositionAngle(){
    setArmAngle(targetAngle);
    //setAngle(targetAngle);
    //setAngle(targetAngle);
  }

  public void dashboardPosition(){

    //rightArmController.setReference(armPosition, ControlType.kPosition);
  }

  public void setMotorsPower(double power){
    leftArm.set(power);
    //rightArm.set(power);

  }
}
