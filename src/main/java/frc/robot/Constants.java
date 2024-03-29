// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean tuningMode = false;

  public static final boolean needToLog = true; 
  
  public static final class DriveConstants{
    public static final int LeftLeader_ID = 2; //3
    public static final int LeftFollower_ID = 1; //4

    public static final int RightLeader_ID = 4; //1
    public static final int RightFollower_ID = 3;  //2

    public static final byte Gyro_ID = 13;

    //Turn PID values, for auto-aligning
    public static double TkP = 0.03,
                               TkI = 0.0,
                               TkD = 0.00245,
                               turnMaxVel = 200,
                               turnMaxAcc = 200;

    //
    public static final double kP = -0.250
    ,  kI = 0
    , kD = 0;

    public static final double kS = 0.16744,//0.15756,
                               kV = 2.1196,//2.0967,
                               kA = 0.80908;//0.69012;

    public static final double WHEEL_DIAMETER = 0.1524;
    public static final double TRACK_WIDTH = 0.504; 
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double TRACK_WIDTH_INCHES = Units.metersToInches(TRACK_WIDTH);

    public static final double TRACK_SCRUB_FACTOR = 0.5;//Para un giro mas cerrado del cheezy drive
    
}

  public static final class ShooterConstants {
    /** ---PROTOTYPE--- */
    public static final byte LEFT_FLYWHEEL_ID = 8; 
    public static final byte RIGHT_FLYWHEEL_ID = 9;
    public static final byte HORIZONTAL_FLYWHEEL_ID = 10;

    public static double kP= 0.087,
                        kI = 0.0,
                        kD = 0.95,
                        kIz = 0,
                        kFF = 0.0495,
                        hKp = 0.00004,
                        hKd = 0.0,
                        hKff = 0.000205;

    public static final double shooterTreshold = 300;

  }

  public static final class ArmConstants {
    public static byte LEFT_ARM_ID = 5; 
    public static byte RIGHT_ARM_ID = 6;

    public static double ARM_DEGREES_PER_MOTOR_ROTATION = 32.89;

    public static double kP = 0.13821,//0.0011773,//0.012904,//4.3755E-09,
                         kI = 0.0,
                         kD = 0.021835,//0.00017643,//0.0024401,//8.274E-10,
                         kFF = 0.0,//0.000156,
                         kMaxVelocityRadPerSecond = 320,
                         kMaxAccelerationMetersPerSecondSquared = 250,//320,
                         kS = 0.94615,//0.82172,
                         kV = 0.0021715,//0.0047927,
                         kA = 0.0019641,//0.003212,
                         kG = 0.12588;//0.44033;

    public static double IDLE_POSITION = 90.0;
    public static double SUBSTATION_POSITION = 130.5;
    public static double SCORING_POSITION = 100;//115;
    public static double BACK_FLOOR_POSITION = 1.15;
    public static double FRONT_FLOOR_POSITION = 175.0;
    public static double AVE_MARIA_SHOOT_POSITION = 120.0;
    public static double COUNTER_BALANCE_POSITION = 60.0;
    public static double TESTING_ANGLE = 90.0;

    public static double ARM_THRESHOLD = 9.5;
  }

  public static final class WristConstants {
    public static byte WRIST_ID = 7; 

    public static double kP = 0.18221,
                         kI = 0.0,
                         kD = 0.0151835,
                         kFF = 0.0,
                         kMaxVelocityRadPerSecond = 440,
                         kMaxAccelerationMetersPerSecondSquared = 440,
                         kS = 0.46147,
                         kV = 0.020646,
                         kA = 0.008245,
                         kG = 0.45406;

    public static double LEFT_POSITION = 90.0;
    public static double IDLE_POSITION = 0.0;
    public static double RIGHT_POSITION = -95.0;

    public static double WRIST_THRESHOLD = 4;
  }


  public static final class AutoConstants{
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
  }

  public static final class VisionConstants {

    public static final String tapeLimelight = "limelight-tape";
    public static final String tagLimelightName = "limelight-tags";

    public static double HEIGHT_OF_HIGH_NODE = 0.90;//Altura del target
    public static double HEIGHT_OF_MID_NODE = 0.60;
    public static double HEIGHT_OF_TAG = 0.45;
    public static double LIMELIGHT_FLOOR_CLEREANCE= 0.04819;//Altura de la limelight
    public static double LIMELIGHT_VERTICAL_ANGLE = 0;

    public static final int normalTracking_Pipeline = 0;
    public static final int lowAlign_Pipeline = 1;
    public static final int midAlign_Pipeline = 2;
    public static final int highAlign_Pipeline = 3;

  }

  public static final class FieldConstants{
    public static final double length = 16.54175;
    public static final double width = 8.0137;
  
    public static final double GRID_CENTER = 2.75;
  
    public static final double CHARGING_STATION_CLEARENCE = 3.00;

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
  
    //Single Substation around = X 14.01 Y 6.83
    public static final Map<String, Pose2d> BLUE_MAP = Map.ofEntries(
            Map.entry("Node 1 Cone", new Pose2d(new Translation2d(1.55, 0.42), Rotation2d.fromDegrees(180))),
            Map.entry("Node 2 Cube", new Pose2d(new Translation2d(1.55, 1.06), Rotation2d.fromDegrees(180))),
            Map.entry("Node 3 Cone", new Pose2d(new Translation2d(1.55, 1.62), Rotation2d.fromDegrees(180))),
            Map.entry("Node 4 Cone", new Pose2d(new Translation2d(1.55, 2.19), Rotation2d.fromDegrees(180))),
            Map.entry("Node 5 Cube", new Pose2d(new Translation2d(1.55, 2.75), Rotation2d.fromDegrees(180))),
            Map.entry("Node 6 Cone", new Pose2d(new Translation2d(1.55, 3.31), Rotation2d.fromDegrees(180))),
            Map.entry("Node 7 Cone", new Pose2d(new Translation2d(1.55, 3.86), Rotation2d.fromDegrees(180))),
            Map.entry("Node 8 Cube", new Pose2d(new Translation2d(1.55, 4.42), Rotation2d.fromDegrees(180))),
            Map.entry("Node 9 Cone", new Pose2d(new Translation2d(1.55, 4.99), Rotation2d.fromDegrees(180)))
          );

          private static final Map<String, Pose2d> RED_MAP =
              BLUE_MAP.entrySet().stream().collect(Collectors.toMap(
                  entry -> entry.getKey(),
                  entry -> new Pose2d(
                      new Translation2d(
                          entry.getValue().getX(),
                          FIELD_WIDTH_METERS - entry.getValue().getY()),
                      entry.getValue().getRotation())));
          
          public static final Map<Alliance, Map<String, Pose2d>> POSE_MAP = Map.of(
              Alliance.Blue, BLUE_MAP,
              Alliance.Red, RED_MAP
          );
  
          public static Map<String, Pose2d> getBlueMap(){
            return BLUE_MAP;
          }
          public static Map<String, Pose2d> getRedMap(){
            return RED_MAP;
          }
    }
}
/*Cherry was here (He left the area) */