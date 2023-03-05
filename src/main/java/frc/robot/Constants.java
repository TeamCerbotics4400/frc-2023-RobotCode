// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team4400.Alert;
import team4400.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  private static final RobotType robot = RobotType.ROBOT_2023P;
  public static final double loopPeriodSecs = 0.02;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2023C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2023P, "/media/sda2/");

  public static enum RobotType {
    ROBOT_2023C,
    ROBOT_2023P,
    ROBOT_SIMBOT
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }


  public static final boolean tuningMode = false;

  
  public static final class DriveConstants{
    public static final int LeftLeader_ID = 2; //3
    public static final int LeftFollower_ID = 1; //4

    public static final int RightLeader_ID = 4; //1
    public static final int RightFollower_ID = 3;  //2

    public static final byte Gyro_ID = 13;

    public static double TkP = 0.03,
                               TkI = 0.0,
                               TkD = 0.004;

    public static final double kP = 0.0039666//0.0028046//4.068E-06,//0.0019356//0.013687
    ,  kI = 0
    , kD = 0;//0.20844;//0.0010903

    public static final double kS = 0.15286,//0.33144,//0.18404,
                               kV = 2.7025,//2.34,//2.7305,
                               kA = 0.69047;//1.1089;//0.83198;

    public static final double WHEEL_DIAMETER = 0.1524;
    public static final double GEAR_RATIO = 0;//7 : 1
    public static final double TRACK_WIDTH = 0.69; 
    public static final double ENCODER_CPR = 0;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double TRACK_WIDTH_INCHES = Units.metersToInches(TRACK_WIDTH);

    public static final double TRACK_SCRUB_FACTOR = 0;//Para un giro mas cerrado del cheezy drive
    
}

  public static final class ShooterConstants {
    /** ---PROTOTYPE--- */
    public static final byte LEFT_FLYWHEEL_ID = 8; 
    public static final byte RIGHT_FLYWHEEL_ID = 9;

    public static double kP= 0.096, //con bandas = 0.098
                        kI = 0.001,
                        kD = 0.001,
                        kIz = 300,
                        kFF = 0.40,//con bandas = 0.42
                        kMaxOutput = 1,
                        kMinOutput = -1, 
                        maxRPM = 0, 
                        maxVel = 0, //---------
                        minVel = 0, //---------
                        maxAcc = 0, 
                        allowedErr = 0;

    public static double targetVelocity = 0;

    public static final double shooterTreshold = 300;

  }

  public static final class ArmConstants {
    public static byte LEFT_ARM_ID = 5; 
    public static byte RIGHT_ARM_ID = 6;

    public static double ARM_DEGREES_PER_MOTOR_ROTATION = 32.89;

    public static double kP = 0.0011773,//0.012904,//4.3755E-09,
                         kI = 0.0,
                         kD = 0.00017643,//0.0024401,//8.274E-10,
                         kFF = 0.0,//0.000156,
                         kMaxVelocityRadPerSecond = 3,
                         kMaxAccelerationMetersPerSecondSquared = 10,
                         //kS = //0.94615,//0.82172,
                         kV = 0.0021715,//0.0047927,
                         kA = 0.0019641,//0.003212,
                         kG = 0.12588;//0.44033;

    public static double OFFSET_DEGREES = 0.0;
  }

  public static final class WristConstants {
    public static byte WRIST_ID = 7; //PlaceHolder

    public static double kP = 0.0,
                         kI = 0.0,
                         kD = 0.0,
                         kFF = 0.0;
  }

  public static final class AutoConstants{
    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
  }

  public static final class VisionConstants {
    /** ---PROTOTYPE--- */
    public static double HEIGHT_OF_OUTER_PORT = 2.64;//Altura del target
    public static double LIMELIGHT_FLOOR_CLEREANCE= 0.79;//Altura de la limelight
    public static double LIMELIGHT_VERTICAL_ANGLE = 36; //Angulo de la limelight

    public static final Transform3d orangeCamPose =
          new Transform3d(
                  new Translation3d(0.2444, 0, 0.454),
                  new Rotation3d(
                          0, 0,
                          0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final String cameraName = "OV9281";
  }

  public static final class FieldConstants{
    public static final double length = 16.54175;
    public static final double width = 8.0137;
  
    public static final double GRID_CENTER = 2.75;
  
    public static final double CHARGING_STATION_CLEARENCE = 3.00;
  
    public static final double FIELD_WIDTH = Units.inchesToMeters((12 * 26) + 3.5);
  
    public static final Map<String, Pose2d> BLUE_MAP = Map.ofEntries(
              Map.entry("No Node", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180))),
              Map.entry("Node 1", new Pose2d(new Translation2d(1.15, 0.42), Rotation2d.fromDegrees(180))),
              Map.entry("Node 2", new Pose2d(new Translation2d(1.15, 1.06), Rotation2d.fromDegrees(180))),
              Map.entry("Node 3", new Pose2d(new Translation2d(1.15, 1.62), Rotation2d.fromDegrees(180))),
              Map.entry("Node 4", new Pose2d(new Translation2d(1.15, 2.19), Rotation2d.fromDegrees(180))),
              Map.entry("Node 5", new Pose2d(new Translation2d(1.15, 2.75), Rotation2d.fromDegrees(180))),
              Map.entry("Node 6", new Pose2d(new Translation2d(1.15, 3.30), Rotation2d.fromDegrees(180))),
              Map.entry("Node 7", new Pose2d(new Translation2d(1.15, 3.86), Rotation2d.fromDegrees(180))),
              Map.entry("Node 8", new Pose2d(new Translation2d(1.15, 4.42), Rotation2d.fromDegrees(180))),
              Map.entry("Node 9", new Pose2d(new Translation2d(1.15, 4.99), Rotation2d.fromDegrees(180))),
              Map.entry("Charging Station", new Pose2d(new Translation2d(3.90, 2.70), Rotation2d.fromDegrees(180)))
              /*Map.entry("Node1Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node2Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node3Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node4Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node5Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node6Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node7Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node8Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node9Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node1Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node2Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node3Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node4Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node5Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node6Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node7Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node8Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
              Map.entry("Node9Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)))*/
          );
          private static final Map<String, Pose2d> RED_MAP =
              BLUE_MAP.entrySet().stream().collect(Collectors.toMap(
                  entry -> entry.getKey(),
                  entry -> new Pose2d(
                      new Translation2d(
                          entry.getValue().getX(),
                          FIELD_WIDTH - entry.getValue().getY()),
                      entry.getValue().getRotation())));
          
          public static final Map<Alliance, Map<String, Pose2d>> POSE_MAP = Map.of(
              Alliance.Blue, BLUE_MAP,
              Alliance.Red, RED_MAP
          );
  
          public static Map<String, Pose2d> getMap(){
            return BLUE_MAP;
          }
    }
}
  


