// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class NeoIntake extends SubsystemBase {
<<<<<<< HEAD
  // Creates a new NeoIntake. 

  CANSparkMax I_Should_Be_A_Servo = new CANSparkMax(IntakeConstants.IShouldBeAServo_ID, MotorType.kBrushless);
=======
  /** Creates a new NeoIntake. */

  CANSparkMax IntakePoseMotor = new CANSparkMax(IntakeConstants.IntakePoseMotor_ID, MotorType.kBrushless);
>>>>>>> ca4a1edbf4b99af1cb37529ba51dc5497accb75a
  CANSparkMax RPWheel = new CANSparkMax(IntakeConstants.RapidWheeel_ID, MotorType.kBrushless);

  private RelativeEncoder IntakePoseMotor_Encoder = IntakePoseMotor.getEncoder();
  
  private SparkMaxPIDController IntakePoseMotor_PIDController = IntakePoseMotor.getPIDController();
  private SparkMaxPIDController RPMWheel_PIDController = RPWheel.getPIDController();

  private double targetPosition = 0;
  private double RPVelo = 0;
  private double intakeUndeployed = 0; //Checar este valor de encoder
  private double intakeDeployed = 0;

  public NeoIntake() { 
    IntakePoseMotor.restoreFactoryDefaults();
    RPWheel.restoreFactoryDefaults();

    IntakePoseMotor.setInverted(true);
    RPWheel.setInverted(true);

    IntakePoseMotor.setCANTimeout(10);
    RPWheel.setCANTimeout(10);

<<<<<<< HEAD
    I_Should_Be_A_Servo.setIdleMode(IdleMode.kBrake);
    RPWheel.setIdleMode(IdleMode.kBrake);
=======
    IntakePoseMotor.setIdleMode(IdleMode.kCoast);
    RPWheel.setIdleMode(IdleMode.kBrake);

    IntakePoseMotor_Encoder.setPosition(0);
>>>>>>> ca4a1edbf4b99af1cb37529ba51dc5497accb75a

    int smartMotionSlot = 0;

    IntakePoseMotor_PIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    IntakePoseMotor_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    IntakePoseMotor_PIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    IntakePoseMotor_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    RPMWheel_PIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    RPMWheel_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    RPMWheel_PIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    RPMWheel_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

<<<<<<< HEAD
    SmartDashboard.putNumber("P Gain", IntakeConstants.kP);
    SmartDashboard.putNumber("I Gain", IntakeConstants.kI);
    SmartDashboard.putNumber("D Gain", IntakeConstants.kD);
    SmartDashboard.putNumber("I Zone", IntakeConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", IntakeConstants.kFF);
    SmartDashboard.putNumber("Max Output", IntakeConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", IntakeConstants.kMinOutput);
    SmartDashboard.putNumber("RP Power", RPVelo);
    SmartDashboard.putNumber("Target Arm Position", targetPosd);
=======
    SmartDashboard.putNumber("Intake P Gain", IntakeConstants.kP);
    SmartDashboard.putNumber("Intake I Gain", IntakeConstants.kI);
    SmartDashboard.putNumber("Intake D Gain", IntakeConstants.kD);
    SmartDashboard.putNumber("Intake Iz Zone", IntakeConstants.kIz);
    SmartDashboard.putNumber("Intake FF", IntakeConstants.kFF);
    SmartDashboard.putNumber("Intake Max Output", IntakeConstants.kMaxOutput);
    SmartDashboard.putNumber("Intake Min Output", IntakeConstants.kMinOutput);
    SmartDashboard.putNumber("Intake RP Velocity", RPVelo);
    SmartDashboard.putNumber("Target INTAKE Velocity", targetVelocity);
    
>>>>>>> ca4a1edbf4b99af1cb37529ba51dc5497accb75a

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P INTAKE Gain", 0);
    double i = SmartDashboard.getNumber("I INTAKE Gain", 0);
    double d = SmartDashboard.getNumber("D INTAKE Gain", 0);
    double iz = SmartDashboard.getNumber("I INTAKE Zone", 0);
    double ff = SmartDashboard.getNumber("Feed INTAKE Forward", 0);
    double max = SmartDashboard.getNumber("Max INTAKE Output", 0);
    double min = SmartDashboard.getNumber("Min INTAKE Output", 0);
    double maxV = SmartDashboard.getNumber("Max INTAKE Velocity", 0);
    double minV = SmartDashboard.getNumber("Min INTAKE Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max INTAKE Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop INTAKE Error", 0);
    double targetVelo = SmartDashboard.getNumber("Target INTAKE Velocity", 0);
    double RPWheelVelo = SmartDashboard.getNumber("Target RP Velocity", 0);
<<<<<<< HEAD
    SmartDashboard.putNumber("Posicion del ''IshouldBeAServo''", i_Should_Be_A_Servo_Encoder.getPosition());
=======
    SmartDashboard.putNumber("Velocidad del ''RPMWheel''", IntakePoseMotor_Encoder.getVelocity());
    SmartDashboard.putNumber("PoseIntake Encoder", IntakePoseMotor_Encoder.getPosition());
>>>>>>> ca4a1edbf4b99af1cb37529ba51dc5497accb75a



    // if PID coefficients on SmartDashboard have changed, write new values to controller
<<<<<<< HEAD
    if((p != IntakeConstants.kP)) {   I_Should_Be_A_Servo_PIDController.setP(p);     }
    if((i != IntakeConstants.kI)) {   I_Should_Be_A_Servo_PIDController.setI(i);     }
    if((d != IntakeConstants.kD)) {   I_Should_Be_A_Servo_PIDController.setD(d);     }
    if((iz != IntakeConstants.kIz)) { I_Should_Be_A_Servo_PIDController.setIZone(iz);}
    if((ff != IntakeConstants.kFF)) { I_Should_Be_A_Servo_PIDController.setFF(ff);   }
    if ((targetPosition != targetVelo )) {targetPosition = targetVelo;}
=======
    if((p != IntakeConstants.kP)) {   IntakePoseMotor_PIDController.setP(p);     }
    if((i != IntakeConstants.kI)) {   IntakePoseMotor_PIDController.setI(i);     }
    if((d != IntakeConstants.kD)) {   IntakePoseMotor_PIDController.setD(d);     }
    if((iz != IntakeConstants.kIz)) { IntakePoseMotor_PIDController.setIZone(iz);}
    if((ff != IntakeConstants.kFF)) { IntakePoseMotor_PIDController.setFF(ff);   }
    if ((targetVelocity != targetVelo )) {targetVelocity = targetVelo;}
>>>>>>> ca4a1edbf4b99af1cb37529ba51dc5497accb75a
    if ((RPWheelVelo != RPVelo)) {RPWheelVelo = RPVelo;}
    if((max != IntakeConstants.kMaxOutput) || (min != IntakeConstants.kMinOutput)) { 
      IntakePoseMotor_PIDController.setOutputRange(min, max); 
      IntakeConstants.kMinOutput = min; IntakeConstants.kMaxOutput = max; 
    }
    
    if((maxV != IntakeConstants.maxVel)) { IntakePoseMotor_PIDController.setSmartMotionMaxVelocity(maxV,0); IntakeConstants.maxVel = maxV; }
    if((minV != IntakeConstants.minVel)) { IntakePoseMotor_PIDController.setSmartMotionMinOutputVelocity(minV,0); IntakeConstants.minVel = minV; }
    if((maxA != IntakeConstants.maxAcc)) { IntakePoseMotor_PIDController.setSmartMotionMaxAccel(maxA,0); IntakeConstants.maxAcc = maxA; }
    if((allE != IntakeConstants.allowedErr)) { IntakePoseMotor_PIDController.setSmartMotionAllowedClosedLoopError(allE,0); IntakeConstants.allowedErr = allE; }

  }

  public void SetRapidWheelVelocity(double setPoint){
    IntakePoseMotor_PIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void goToDashboardVelocity(){
    SetRapidWheelVelocity(targetPosition);
    setMotorsPower(RPVelo);
  }

  public void setIntakePosition(double pose){
    IntakePoseMotor_PIDController.setReference(pose, ControlType.kSmartMotion);
  }
<<<<<<< HEAD
=======

  public void setMotorsPower(double Power){
    IntakePoseMotor.set(Power);
  }
>>>>>>> ca4a1edbf4b99af1cb37529ba51dc5497accb75a
}
