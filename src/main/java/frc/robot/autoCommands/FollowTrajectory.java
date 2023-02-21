package frc.robot.autoCommands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectory extends SequentialCommandGroup{
    
    public FollowTrajectory(DriveTrain m_drive, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(m_drive);

        if(resetOdometry) {
            m_drive.resetOdometry(trajectory.getInitialPose());
        }
        
        addCommands(
            m_drive.createCommandForTrajectory(trajectory, null)
        );
    }
    
    public FollowTrajectory(DriveTrain m_drive, Supplier<PathPlannerTrajectory> trajectoryGen, boolean resetOdometry) {
        addRequirements(m_drive);
        
        if(resetOdometry) {
            addCommands(new InstantCommand(() -> m_drive.resetOdometry(trajectoryGen.get().getInitialHolonomicPose())));
        }

        addCommands(m_drive.createCommandForTrajectory(trajectoryGen.get(), false));
        
    }
}
