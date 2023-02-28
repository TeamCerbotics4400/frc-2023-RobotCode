// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class NodeSelector extends SubsystemBase {
  /** Creates a new NodeSelector. */
  private Map<String, Pose2d> pose_map;
  private int currentSelection;
  private ArrayList<String> nodeNames = new ArrayList<String>();
  Joystick joy;

  public static Pose2d nodeToAlign = new Pose2d();

  public NodeSelector(Joystick joy) {
    this.joy = joy;
    this.pose_map = FieldConstants.getMap();
    this.currentSelection = 0;

    nodeNames.add("No Node");
    nodeNames.add("Node 1");
    nodeNames.add("Node 2");
    nodeNames.add("Node 3");
    nodeNames.add("Node 4");
    nodeNames.add("Node 5");
    nodeNames.add("Node 6");
    nodeNames.add("Node 7");
    nodeNames.add("Node 8");
    nodeNames.add("Node 9");
    nodeNames.add("Charging Station");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displaySelection();
  }

  public void updateSelectionRight(){
    int pov = joy.getPOV();

    if(pov == 90){
      currentSelection++;
      if(currentSelection >= nodeNames.size()){
        currentSelection = 0;
      }
    }
  }

  public void updateSelectionLeft(){
    int pov = joy.getPOV();

    if(pov == 270){
      currentSelection--;
      if(currentSelection < 0){
        currentSelection = nodeNames.size() - 1;
      }
    }  
  }

  public void displaySelection(){
    
    String currentKey = nodeNames.get(currentSelection);

    if (currentKey != null) {
      // Get the string representation of the selected entry
  
      // Display the selected entry on the SmartDashboard
      SmartDashboard.putString("Selected Node", currentKey);
      nodeToAlign = pose_map.get(currentKey);
    } else {
      // Display a message indicating that the selected entry is null
      SmartDashboard.putString("Selected Entry", "No node selected");
    }
  }

  public Pose2d getNodeToAlign(){
    return nodeToAlign;
  }
  /* 
  public double getNodeToAlignDistance(Translation2d currentRobotTranslation){
    return pose_map.get(currentKey).getTranslation().getDistance(currentRobotTranslation);
  }

  public Translation2d getNodeToAlignTranslation(){
    return pose_map.get(currentKey).getTranslation();
  }*/
}
