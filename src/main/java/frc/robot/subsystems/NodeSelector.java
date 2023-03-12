// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class NodeSelector extends SubsystemBase {
  /** Creates a new NodeSelector. */
  private Map<String, Pose2d> pose_map;
  private int currentSelectionNodes;
  private int currentSelectionLevels;
  private ArrayList<String> nodeNames = new ArrayList<String>();
  private ArrayList<String> scoringLevels = new ArrayList<String>();
  private String level;
  Joystick joy;

  public static Pose2d nodeToAlign = new Pose2d();

  public static String levelToShoot = new String();

  public NodeSelector(Joystick joy) {
    this.joy = joy;
    this.level = levelToShoot;
    if(DriverStation.getAlliance() == Alliance.Blue){
    this.pose_map = FieldConstants.getBlueMap();
    } else{
    this.pose_map = FieldConstants.getRedMap();
    }
    this.currentSelectionNodes = 0;
    this.currentSelectionLevels = 2;

    nodeNames.add("No Node");
    nodeNames.add("Node 1 Cone");
    nodeNames.add("Node 2 Cube");
    nodeNames.add("Node 3 Cone");
    nodeNames.add("Node 4 Cone");
    nodeNames.add("Node 5 Cube");
    nodeNames.add("Node 6 Cone");
    nodeNames.add("Node 7 Cone");
    nodeNames.add("Node 8 Cube");
    nodeNames.add("Node 9 Cone");
    nodeNames.add("Single Substation");

    scoringLevels.add("Low");
    scoringLevels.add("Mid");
    scoringLevels.add("High");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displaySelection();
  }

  public void updateSelectionRight(){
    int pov = joy.getPOV();

    if(pov == 90){
      currentSelectionNodes++;
      if(currentSelectionNodes >= nodeNames.size()){
        currentSelectionNodes = 0;
      }
    }
  }

  public void updateSelectionLeft(){
    int pov = joy.getPOV();

    if(pov == 270){
      currentSelectionNodes--;
      if(currentSelectionNodes < 0){
        currentSelectionNodes = nodeNames.size() - 1;
      }
    }  
  }

  public void updateSelectionUp(){
    int pov = joy.getPOV();

    if(pov == 0){
      currentSelectionLevels++;
      if(currentSelectionLevels >= scoringLevels.size()){
        currentSelectionLevels = 0;
      }
    }  
  }

  public void updateSelectionDown(){
    int pov = joy.getPOV();

    if(pov == 180){
      currentSelectionLevels--;
      if(currentSelectionLevels < 0){
        currentSelectionLevels = scoringLevels.size() - 1;
      }
    }  
  }

  public void displaySelection(){
    
    String currentKeyNodes = nodeNames.get(currentSelectionNodes);
    String currentKeyLevels = scoringLevels.get(currentSelectionLevels);

    if (currentKeyNodes != null) {
      // Get the string representation of the selected entry
  
      // Display the selected entry on the SmartDashboard
      SmartDashboard.putString("Selected Node", currentKeyNodes);
      nodeToAlign = pose_map.get(currentKeyNodes);

      SmartDashboard.putString("Selected Level", currentKeyLevels);
      levelToShoot = level;

      //nodeToAlign = pose_map.get(currentKeyNodes);
    } else {
      // Display a message indicating that the selected entry is null
      SmartDashboard.putString("Selected Entry", "No node selected");
    }
  }

  public Pose2d getNodeToAlign(){
    return nodeToAlign;
  }

  public String getAlignName(){
    String currentKey = nodeNames.get(currentSelectionNodes);
    return currentKey;
  }

  public String getLevelName(){
    String currentKey = scoringLevels.get(currentSelectionLevels);
    return currentKey;
  }
}
