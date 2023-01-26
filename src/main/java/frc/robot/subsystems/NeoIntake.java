// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class NeoIntake extends SubsystemBase {
  // Creates a new NeoIntake. 

  
  CANSparkMax RapidWheel = new CANSparkMax(IntakeConstants.RapidWheeel_ID, MotorType.kBrushless);
  

  private SparkMaxPIDController RapidWHeelPIDController = RapidWheel.getPIDController();

  private double targetPosition = 0;
  private double RapidVelo = 0;

  //Posiciones Intake.

  private CANSparkMax PositionIntake = new CANSparkMax(IntakeConstants.PositionIntake, MotorType.kBrushless);

  private RelativeEncoder PositionIntake_encoder = PositionIntake.getEncoder();

  private SparkMaxPIDController PositionIntake_PIDController = PositionIntake.getPIDController();

  private double DeployPosition = 0;
  private double UnDeployPosition = IntakeConstants.Undeploy;

  public NeoIntake() { 

    RapidWheel.restoreFactoryDefaults();

    RapidWheel.setInverted(true);

    RapidWheel.setCANTimeout(10);
    RapidWheel.setIdleMode(IdleMode.kBrake);

    int smartMotionSlot = 0;

    RapidWHeelPIDController.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    RapidWHeelPIDController.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    RapidWHeelPIDController.setSmartMotionMaxAccel(IntakeConstants.maxAcc, smartMotionSlot);
    RapidWHeelPIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    SmartDashboard.putNumber("P Gain", IntakeConstants.Kp);
    SmartDashboard.putNumber("I Gain", IntakeConstants.kI);
    SmartDashboard.putNumber("D Gain", IntakeConstants.kD);
    SmartDashboard.putNumber("I Zone", IntakeConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", IntakeConstants.kFF);
    SmartDashboard.putNumber("Max Output", IntakeConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", IntakeConstants.kMinOutput);
    SmartDashboard.putNumber("RP Power", RapidVelo);
    SmartDashboard.putNumber("Set Rotations", targetPosition);

    //Posiciones Intake.

    PositionIntake.restoreFactoryDefaults();
    PositionIntake.setCANTimeout(10);
    PositionIntake.setIdleMode(IdleMode.kBrake);

    PositionIntake_PIDController.setSmartMotionMaxVelocity(IntakeConstants.i_MaxVel, smartMotionSlot);
    PositionIntake_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.i_MinVel, smartMotionSlot);
    PositionIntake_PIDController.setSmartMotionMaxAccel(IntakeConstants.i_MaxAcc, smartMotionSlot);
    PositionIntake_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.i_allowedErr, smartMotionSlot);

    SmartDashboard.putNumber("Intake_Arm P Gain", IntakeConstants.i_kP);
    SmartDashboard.putNumber("Intake_Arm I Gain", IntakeConstants.i_kI);
    SmartDashboard.putNumber("Intake_Arm D Gain", IntakeConstants.i_kD);
    SmartDashboard.putNumber("Intake_Arm I Zone", IntakeConstants.i_kI);
    SmartDashboard.putNumber("Intake_Arm Feed Forward", IntakeConstants.i_kFF);
    SmartDashboard.putNumber("Intake_Arm Max Output", IntakeConstants.i_kMaxOutput);
    SmartDashboard.putNumber("Intake_Arm Min Output", IntakeConstants.i_kMinOutput);
    SmartDashboard.putNumber("Deploy Position", DeployPosition);
    SmartDashboard.putNumber("Deploy Position", IntakeConstants.Undeploy);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);


                String fDeFuck = "f de fuck";
                        if(fDeFuck!= "f de jfuck"){
                          System.out.println("This code is not workin' 'cause the line '''' is not available");
                        }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
  
    //Posiciones Intake.

    double i_p = SmartDashboard.getNumber("Intake_Arm P Gain", 0);
    double i_i = SmartDashboard.getNumber("Intake_Arm I Gain", 0);
    double i_d = SmartDashboard.getNumber("Intake_Arm D Gain", 0);
    double i_iz = SmartDashboard.getNumber("Intake_Arm I Zone", 0);
    double i_ff = SmartDashboard.getNumber("Intake_Arm Feed Forward", 0);
    double i_max = SmartDashboard.getNumber("Intake_Arm Max Output", 0);
    double i_min = SmartDashboard.getNumber("Intake_Arm Min Output", 0);
    double i_maxV = SmartDashboard.getNumber("Intake_ArmMax Velocity", 0);
    double i_minV = SmartDashboard.getNumber("Intake_Arm Min Velocity", 0);
    double i_maxA = SmartDashboard.getNumber("Intake_Arm Max Acceleration", 0);
    double i_allE = SmartDashboard.getNumber("Intake_Arm Allowed Closed Loop Error", 0);

    if((p != IntakeConstants.i_kP)) { PositionIntake_PIDController.setP(p); IntakeConstants.i_kP = i_p; }
    if((i != IntakeConstants.i_kI)) { PositionIntake_PIDController.setI(i); IntakeConstants.i_kI = i_i; }
    if((d != IntakeConstants.i_kD)) { PositionIntake_PIDController.setD(d); IntakeConstants.i_kD = i_d; }
    if((iz != IntakeConstants.i_kIz)) { PositionIntake_PIDController.setIZone(iz); IntakeConstants.i_kIz = i_iz; }
    if((ff != IntakeConstants.i_kFF)) { PositionIntake_PIDController.setFF(ff); IntakeConstants.i_kFF = i_ff; }
    if((max != IntakeConstants.i_kMaxOutput) || (i_min != IntakeConstants.i_kMinOutput)) { 
      PositionIntake_PIDController.setOutputRange(i_min, i_max); 
      IntakeConstants.i_kMinOutput = i_min; IntakeConstants.i_kMaxOutput = i_max; 
    }
    if((i_maxV != IntakeConstants.i_MaxVel)) { PositionIntake_PIDController.setSmartMotionMaxVelocity(i_maxV,0); IntakeConstants.i_MaxVel = i_maxV; }
    if((i_minV != IntakeConstants.i_MinVel)) { PositionIntake_PIDController.setSmartMotionMinOutputVelocity(i_minV,0); IntakeConstants.i_MinVel = i_minV; }
    if((i_maxA != IntakeConstants.i_MaxAcc)) { PositionIntake_PIDController.setSmartMotionMaxAccel(i_maxA,0); IntakeConstants.i_MaxAcc = i_maxA; }
    if((i_allE != IntakeConstants.i_allowedErr)) { PositionIntake_PIDController.setSmartMotionAllowedClosedLoopError(i_allE,0); IntakeConstants.i_allowedErr = i_allE; }

    double deploy_setPoint, deploy_processVariable;
    boolean deploy_mode = SmartDashboard.getBoolean("Deploy Mode", false);
    if(deploy_mode) {
      deploy_setPoint = SmartDashboard.getNumber("Set Position UP", 0);
      PositionIntake_PIDController.setReference(deploy_setPoint, CANSparkMax.ControlType.kSmartMotion);
      deploy_processVariable = PositionIntake_encoder.getPosition();
    } else {
      deploy_setPoint = SmartDashboard.getNumber("Position", 0);
      /**
       /*As with other PID modes, Smart Motion is set by calling the
       /** setReference method on an existing pid object and setting
       /* the control type to kSmartMotion
       /*/
      PositionIntake_PIDController.setReference(deploy_setPoint, CANSparkMax.ControlType.kSmartMotion);
      deploy_processVariable = PositionIntake_encoder.getPosition();

      
    }

    SmartDashboard.putNumber("i_SetPoint", deploy_setPoint);
    SmartDashboard.putNumber("i_Process Variable", deploy_processVariable);
    SmartDashboard.putNumber("i_Output", PositionIntake.getAppliedOutput());    

  }
}
