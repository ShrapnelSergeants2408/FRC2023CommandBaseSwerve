// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_robotDrive = new Drivetrain();

  // The driver's controller
  
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_driverController.getLeftY(),
                    m_driverController.getLeftX(),
                    m_driverController.getRightX(),
                    true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
        
    JoystickButton m_driverLeft = new JoystickButton(m_driverController, OIConstants.kD_Left);
    JoystickButton m_driverRight = new JoystickButton(m_driverController, OIConstants.kD_Right);
    JoystickButton m_driverUp = new JoystickButton(m_driverController, OIConstants.kD_Up);
    JoystickButton m_driverDown = new JoystickButton(m_driverController, OIConstants.kD_Down);
    JoystickButton m_driverShoulderTopLeft = new JoystickButton(m_driverController, OIConstants.kD_Shoulder_Top_Left);
    JoystickButton m_driverShoulderTopRight = new JoystickButton(m_driverController, OIConstants.kD_Shoulder_Top_Right);
    JoystickButton m_driverShoulderBottomLeft = new JoystickButton(m_driverController, OIConstants.kD_Shoulder_Bottom_Left);
    JoystickButton m_driverShoulderBottomRight = new JoystickButton(m_driverController, OIConstants.kD_Shoulder_Bottom_Right);
    JoystickButton m_driverLeftJoystick = new JoystickButton(m_driverController, OIConstants.kD_Left_Joystick);
    JoystickButton m_driverRightJoystick = new JoystickButton(m_driverController, OIConstants.kD_Right_Joystick);

    JoystickButton m_operatorLeft = new JoystickButton(m_operatorController,OIConstants.kO_Left);
    JoystickButton m_operatorRight = new JoystickButton(m_operatorController,OIConstants.kO_Right);
    JoystickButton m_operatorUp = new JoystickButton(m_operatorController,OIConstants.kO_Up);
    JoystickButton m_operatorDown = new JoystickButton(m_operatorController,OIConstants.kO_Down);
    JoystickButton m_operatorShoulderTopLeft = new JoystickButton(m_operatorController,OIConstants.kO_Shoulder_Top_Left);
    JoystickButton m_operatorShoulderTopRight = new JoystickButton(m_operatorController,OIConstants.kO_Shoulder_Top_Right);
    JoystickButton m_operatorShoulderBottomLeft = new JoystickButton(m_operatorController,OIConstants.kO_Shoulder_Bottom_Left);
    JoystickButton m_operatorShoulderBottomRight = new JoystickButton(m_operatorController,OIConstants.kO_Shoulder_Bottom_Right);
    //JoystickButton m_operatorMidLeft = new JoystickButton(m_operatorController,OIConstants.kO_);
    JoystickButton m_operatorLeftJoystick = new JoystickButton(m_operatorController,OIConstants.kO_Left_Joystick);
    JoystickButton m_operatorRightJoystick = new JoystickButton(m_operatorController,OIConstants.kO_Right_Joystick);

    //button command links
    //m_operatorUp.whenPressed(new ShootHigh(shooter)); //set shooter motor to shoot to high goal
    //operatorUp.whenReleased(new ShootOff(shooter)); //turn off shooter motor
    m_operatorUp.onTrue(m_robotDrive.doNothing());

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
