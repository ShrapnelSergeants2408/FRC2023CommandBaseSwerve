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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhysicalConstants;

import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.GrabPiece;
import frc.robot.commands.ReleasePiece;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.Autonomous.DoNothing;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final Gripper m_robotGripper = new Gripper();

  // The autonomous routines

  // A simple auto routine that does nothing
  private final Command m_doNothing = new DoNothing(m_robotDrive);



  // The driver's controller
  
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_robotDrive.setDefaultCommand(new DriveWithJoysticks(
      m_robotDrive, 
      () -> -m_driverController.getLeftY()*DriveConstants.kDriveSpeedMultiplier,
      () -> m_driverController.getLeftX()*DriveConstants.kDriveSpeedMultiplier,
      () -> m_driverController.getRightX()*DriveConstants.kTurnSpeedMultiplier, 
      () -> !m_driverController.getRawButton(OIConstants.kD_Mid_Left)));

    m_robotGripper.setDefaultCommand(
        // default to gripper closed
        new RunCommand(
            () ->
                m_robotGripper.grabPiece(),
        m_robotGripper)
    );  

    // Add commands to the autonomous command chooser
    m_AutoChooser.setDefaultOption("Do Nothing", m_doNothing);
    m_AutoChooser.addOption("First Auto", trajectory1Command());
    m_AutoChooser.addOption("Second Auto", trajectory2Command());
    m_AutoChooser.addOption("Third Auto", trajectory3Command());

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_AutoChooser);
  

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
    JoystickButton m_driverMidLeft = new JoystickButton(m_driverController,OIConstants.kD_Mid_Left);
    JoystickButton m_driverMidRight = new JoystickButton(m_driverController, OIConstants.kD_Mid_Right);
    //JoystickButton m_driverPOV = new JoystickButton(m_driverController, )

    JoystickButton m_operatorLeft = new JoystickButton(m_operatorController,OIConstants.kO_Left);
    JoystickButton m_operatorRight = new JoystickButton(m_operatorController,OIConstants.kO_Right);
    JoystickButton m_operatorUp = new JoystickButton(m_operatorController,OIConstants.kO_Up);
    JoystickButton m_operatorDown = new JoystickButton(m_operatorController,OIConstants.kO_Down);
    JoystickButton m_operatorShoulderTopLeft = new JoystickButton(m_operatorController,OIConstants.kO_Shoulder_Top_Left);
    JoystickButton m_operatorShoulderTopRight = new JoystickButton(m_operatorController,OIConstants.kO_Shoulder_Top_Right);
    JoystickButton m_operatorShoulderBottomLeft = new JoystickButton(m_operatorController,OIConstants.kO_Shoulder_Bottom_Left);
    JoystickButton m_operatorLeftJoystick = new JoystickButton(m_operatorController,OIConstants.kO_Left_Joystick);
    JoystickButton m_operatorRightJoystick = new JoystickButton(m_operatorController,OIConstants.kO_Right_Joystick);
    


    //button command links
 
    m_operatorUp.onTrue(m_robotArm.setArmGoalCommand(2)); //move to 2 rad
    m_operatorDown.onTrue(m_robotArm.setArmGoalCommand(1)); //move to 1 rad
    
    //m_operatorShoulderTopRight.onTrue(new GrabPiece(m_robotGripper));
    //m_operatorShoulderTopLeft.onTrue(new ReleasePiece(m_robotGripper));

    // turn to angle specified by driver controller POV hat.  Currently overrides driveWithJoysticks
    if (m_driverController.getPOV() != -1){
      new TurnToAngle(m_driverController.getPOV(), m_robotDrive);
    }

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   
  public Command getAutonomousCommand() {

    return m_AutoChooser.getSelected();

  }
  
  public Command trajectory1Command() {
    /* Sample trajectory. 
        Start at origin facing +X direction
        Pass through (1,1) (1 meter forward, 1 meter left of start)
        Pass through (2, -1) (2 meters forward, 1 meter right of start)
        End at (3, 0) (3 meters forward, even with start)
        Throughout path rotate 45 degrees left    
    */

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(PhysicalConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(45)),
            config);

    var thetaController =
    
        new ProfiledPIDController(
            AutoConstants.kPThetaController,
            AutoConstants.kIThetaController,
            AutoConstants.kDThetaController,
            AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            PhysicalConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController,
                              AutoConstants.kIXController,
                              AutoConstants.kDXController),
            new PIDController(AutoConstants.kPYController,
                              AutoConstants.kIYController,
                              AutoConstants.kDYController),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    
  }

  public Command trajectory2Command() {
        /*Sample trajectory. 
        Start at origin facing +X direction
        Pass through (3,0) (3 meter forward, even with start)
        Pass through (3,2) (3 meters forward, 2 meter left of start)
        Pass through (0, 2) (0 meters forward, 2 meters left of start)
        End at origin facing -X direction
        
         */
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(PhysicalConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(3, 0),
                        new Translation2d(3, 2),
                        new Translation2d(0,2)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
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
                m_robotDrive::getPose,
                PhysicalConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // 5. Add some init and wrap-up, and return everything
        //return new SequentialCommandGroup(
        //        new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory.getInitialPose())),
        //        swerveControllerCommand,
        //        new InstantCommand(() -> m_robotDrive.stopModules()));
        return new InstantCommand(
            ()-> m_robotDrive.resetOdometry(trajectory.getInitialPose()))
            .andThen(swerveControllerCommand)
            .andThen(new InstantCommand(() -> m_robotDrive.stopModules()));
  }

  public Command trajectory3Command() {
    /*Sample trajectory PID test
      Drive forward 1 meter (? meters?  feet?  inches?)
     */
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(PhysicalConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(.25, 0),
                    new Translation2d(.50, 0),
                    new Translation2d(.75,0)),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
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
            m_robotDrive::getPose,
            PhysicalConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // 5. Add some init and wrap-up, and return everything
    //return new SequentialCommandGroup(
    //        new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory.getInitialPose())),
    //        swerveControllerCommand,
    //        new InstantCommand(() -> m_robotDrive.stopModules()));
    return new InstantCommand(
        ()-> m_robotDrive.resetOdometry(trajectory.getInitialPose()))
        .andThen(swerveControllerCommand)
        .andThen(new InstantCommand(() -> m_robotDrive.stopModules()));
  }
}
