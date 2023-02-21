// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Trajectories.TestAlignTrajectory;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithTag extends SequentialCommandGroup {
  /** Creates a new AlignWithTag. */
  public AlignWithTag(DriveTrain m_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var ResetOdometry = new InstantCommand(() -> 
      m_drive.resetPoseWVision());

    addCommands(ResetOdometry, 
    m_drive.createCommandForTrajectory(TestAlignTrajectory.generatedTrajectory(m_drive),
     false).andThen(() -> m_drive.tankDriveVolts(0, 0)));
  }
}
