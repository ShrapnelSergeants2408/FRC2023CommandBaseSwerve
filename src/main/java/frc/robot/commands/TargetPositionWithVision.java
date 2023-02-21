// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * On driver shoulder button press check distance to target.
 * If distance less than 2 meters, engage auto targeting
 * 
 * Compare target pose (37.8" in front of aprilTag)(2 feet behind scoring line)
 * to current estimated pose.
 * 
 * Create trajectory (no waypoints?) to transform current pose to target pose.
 * 
 */

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

public class TargetPositionWithVision extends CommandBase {
  /** Creates a new TargetPositionWithVision. */
  Drivetrain m_driveTrain;
  public TargetPositionWithVision(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = drivetrain;

    addRequirements(m_driveTrain); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //calculate distance to best target

     double range = PhotonUtils
        .getDistanceToPose(
        m_driveTrain.getPose(),
        m_driveTrain.getTargetPose()
        );

      //only engage auto target within 2 meters
      if(range < VisionConstants.kAutoTargetRange) { 
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(PhysicalConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                m_driveTrain.getPose(),
                List.of(),
                m_driveTrain.getTargetPose(), //TODO: add/subtract 38" to x value
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 
                                                      AutoConstants.kIXController,
                                                      AutoConstants.kDXController);
        PIDController yController = new PIDController(AutoConstants.kPYController,
                                              AutoConstants.kIYController,
                                              AutoConstants.kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                                        AutoConstants.kPThetaController,
                                        AutoConstants.kIThetaController,
                                        AutoConstants.kDThetaController,
                                        AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_driveTrain::getPose,
                PhysicalConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_driveTrain::setModuleStates,
                m_driveTrain);

        //TODO: can i delete this
        // 5. Add some init and wrap-up, and return everything
/*
        return new InstantCommand(
            ()-> m_robotDrive.resetOdometry(trajectory.getInitialPose()))
            .andThen(swerveControllerCommand)
            .andThen(new InstantCommand(() -> m_robotDrive.stopModules()));
*/
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
