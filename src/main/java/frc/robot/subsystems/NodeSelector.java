// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class NodeSelector extends SubsystemBase {
  /** Creates a new NodeSelector. */
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
    this.currentSelectionNodes = 0;
    this.currentSelectionLevels = 2;

    nodeNames.add("Node 1 Cone");
    nodeNames.add("Node 2 Cube");
    nodeNames.add("Node 3 Cone");
    nodeNames.add("Node 4 Cone");
    nodeNames.add("Node 5 Cube");
    nodeNames.add("Node 6 Cone");
    nodeNames.add("Node 7 Cone");
    nodeNames.add("Node 8 Cube");
    nodeNames.add("Node 9 Cone");

    scoringLevels.add("Low");
    scoringLevels.add("Mid");
    scoringLevels.add("High");
    scoringLevels.add("Ave Maria");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displaySelection();
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
    
    String currentKeyLevels = scoringLevels.get(currentSelectionLevels);

    if (currentKeyLevels != null) {
      // Get the string representation of the selected entry
  
      // Display the selected entry on the SmartDashboard
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

  public void selectNode(int node){
    currentSelectionNodes = node;
  }

  public void selectLevel(int level){
    currentSelectionLevels = level;
  }
}
