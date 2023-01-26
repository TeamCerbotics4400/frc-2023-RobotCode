// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. 

  private static LimelightSubsystem instance = null;
  
  private DoubleArraySubscriber camposeSub;
  private DoubleArraySubscriber posesub;
  
  private Translation3d tran3d;
  private Rotation3d r3d;
  private Pose3d p3d;
  //double[] result = new double[6];

  
  // Filters to prevent target values from oscillating too much
SlewRateLimiter targetXFilter = new SlewRateLimiter(20);
SlewRateLimiter innerTargetXFilter = new SlewRateLimiter(20);
NetworkTable limelight;
NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    

  public LimelightSubsystem() {
    /*result[0] = 0;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0;
    result[4] = 0;
    result[5] = 0;
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    ta = limelight.getEntry("ta");
    camposeSub = limelight.getDoubleArrayTopic("campose").subscribe(new, null)
    posesub = limelight.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
     
    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
    SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));// Datos del target
    //SmartDashboard.putNumber("Distancia X vision", getRobotPose().getX());
    //SmartDashboard.putNumber("Distancia Y vision", getRobotPose().getY());
    //`SmartDashboard.putNumber("Angle vision", getRobotPose().getRotation().getAngle());

  
    distFromTarget();
  }

  public double distFromTarget() {
    double distance = (Constants.VisionConstants.HEIGHT_OF_OUTER_PORT
    - Constants.VisionConstants.LIMELIGHT_FLOOR_CLEREANCE)
        / Math.tan(Math.toRadians(ty.getDouble(0.0) + Constants.VisionConstants.LIMELIGHT_VERTICAL_ANGLE));
    
        SmartDashboard.putNumber("distance from target", distance);
    return distance ;
  }

  public static LimelightSubsystem getInstance() {
    return instance == null ? instance = new LimelightSubsystem() : instance;
  }

  // Limelight interaction functions
  public double getTargetY() {
    return limelight.getEntry("ty").getDouble(0);
  }
  
  public double getTargetX() {
    return limelight.getEntry("tx").getDouble(0);
  }
  
  public double getFilteredTargetX() {
    return targetXFilter.calculate(getTargetX());
  }
  
  // More Limelight interaction functions
  
  public boolean hasTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1;
  }
  
  public double getTargetArea() {
    return limelight.getEntry("ta").getDouble(0);
  }
  
  public double getTargetSkew() {
    return limelight.getEntry("ts").getDouble(0);
  }
  
  public double getPipelineLatency() {
    return limelight.getEntry("tl").getDouble(0);
  }

  
  public Pose3d getRobotPose(){
    
    double[] result = posesub.get();
    tran3d = new Translation3d(result[0], result[1], result[2]);
    r3d = new Rotation3d(result[3], result[4], result[5]);
    p3d = new Pose3d(tran3d, r3d);
    
        return p3d;
    }
    
  
  public double getTargetShort() {
    return limelight.getEntry("tshort").getDouble(0);
  }
  
  public double getTargetLong() {
    return limelight.getEntry("tlong").getDouble(0);
  }
  
  public double getHorizontalSidelength() {
    return limelight.getEntry("thor").getDouble(0);
  }
  
  public double getVerticalSidelength() {
    return limelight.getEntry("tvert").getDouble(0);
  }
  
  public double getPipeline() {
    return limelight.getEntry("getpipe").getDouble(0);
  }
  
  public void getStream(){
  }
  public void ledsOn() {
    limelight.getEntry("ledMode").setNumber(3);
  }
  
  public void ledsOff() {
    limelight.getEntry("ledMode").setNumber(1);
  }
  
  public void setPipeline(int pipeline) {
    limelight.getEntry("pipeline").setNumber(pipeline);
  }
  

}*/