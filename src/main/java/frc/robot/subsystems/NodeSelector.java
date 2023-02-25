// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class NodeSelector extends SubsystemBase {
  /** Creates a new NodeSelector. */
  private Map<String, Pose2d> pose_map;
  private List<Map.Entry<String, Pose2d>> entryList;
  private int currentSelection;
  Joystick joy;

  public NodeSelector(Joystick joy) {
    this.joy = joy;
    this.pose_map = FieldConstants.getMap();
    this.currentSelection = 0;
    entryList = new ArrayList<>(pose_map.entrySet());
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
      if(currentSelection >= entryList.size()){
        currentSelection = 0;
      }
    }
  }

  public void updateSelectionLeft(){
    int pov = joy.getPOV();

    if(pov == 270){
      currentSelection--;
      if(currentSelection < 0){
        currentSelection = entryList.size() - 1;
      }
    }  
  }



  public void displaySelection(){
    
    String currentKey = (String) pose_map.keySet().toArray()[currentSelection];

    if (currentKey != null) {
      // Get the string representation of the selected entry
  
      // Display the selected entry on the SmartDashboard
      SmartDashboard.putString("Selected Node", currentKey);
  } else {
      // Display a message indicating that the selected entry is null
      SmartDashboard.putString("Selected Entry", "No node selected");
  }
  }
}
