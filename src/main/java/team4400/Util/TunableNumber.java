// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4400.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */

public class TunableNumber {
    private String key;
    private double defaultValue;
  
    /**
     * Create a new TunableNumber
     * 
     * @param dashboardKey Key on dashboard
     */
    public TunableNumber(String dashboardKey) {
      this.key = dashboardKey;
    }
  
    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault() {
      return defaultValue;
    }
  
    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue) {
      this.defaultValue = defaultValue;
      if (Constants.tuningMode) {
        // This makes sure the data is on NetworkTables but will not change it
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
      }
    }
  
    /**
     * Get the current value, from dashboard if available and in tuning mode
     * 
     * @return The current value
     */
    public double get() {
      return Constants.tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }
  }