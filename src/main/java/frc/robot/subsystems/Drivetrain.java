// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.DriveConstants.*;
import frc.robot.Constants.PhysicalConstants;

import java.io.IOException;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
//import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.VisionConstants;
//import frc.robot.lib.PhotonCameraWrapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Drivetrain extends SubsystemBase {
  // Robot swerve modules
    /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor SparkMax
   * @param turningMotorChannel The channel of the turning motor VictorSPX
   * @param driveMotorReversed Is the drive motor reversed
   * @param turningMotorReversed Is the turning motor reversed
   * @param turningEncoderChannel ANA port of MA3 turning encoder
   * @param turningEncoderOffset calculated value for offset
   * @param turningEncoderReversed is the turning encoder reversed
   */
//+
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          kFrontLeftDriveMotorPort,
          kFrontLeftTurningMotorPort,
          kFrontLeftDriveMotorReversed,
          kFrontLeftTurningMotorReversed,
          kFrontLeftTurningEncoderPorts,
          kFrontLeftTurningEncoderOffset,
          kFrontLeftTurningEncoderReversed
          );
//+
  private final SwerveModule m_rearLeft =
      new SwerveModule(
          kRearLeftDriveMotorPort,
          kRearLeftTurningMotorPort,
          kRearLeftDriveMotorReversed,
          kRearLeftTurningMotorReversed,
          kRearLeftTurningEncoderPorts,
          kRearLeftTurningEncoderOffset,
          kRearLeftTurningEncoderReversed
          );
//+
  private final SwerveModule m_frontRight =
      new SwerveModule(
          kFrontRightDriveMotorPort,
          kFrontRightTurningMotorPort,
          kFrontRightDriveMotorReversed,
          kFrontRightTurningMotorReversed,
          kFrontRightTurningEncoderPorts,
          kFrontRightTurningEncoderOffset,
          kFrontRightTurningEncoderReversed
          );
//+
  private final SwerveModule m_rearRight =
      new SwerveModule(
          kRearRightDriveMotorPort,
          kRearRightTurningMotorPort,
          kRearRightDriveMotorReversed,
          kRearRightTurningMotorReversed,
          kRearRightTurningEncoderPorts,
          kRearRightTurningEncoderOffset,
          kRearRightTurningEncoderReversed
          );
//+
  // The navX MXP gyro sensor
  private final AHRS m_gyro = new AHRS();

//-
  //vision stuff
  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonPoseEstimator;

  private Pose2d targetPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private Pose2d defaultPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

//+
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          PhysicalConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

//-          
  /*
  * Here we use SwerveDrivePoseEstimator so that we can fuse odometry
  * readings. 
  */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(PhysicalConstants.kDriveKinematics, 
                                   m_gyro.getRotation2d(), 
                                   new SwerveModulePosition[] {
                                      m_frontLeft.getPosition(),
                                      m_frontRight.getPosition(),
                                      m_rearLeft.getPosition(),
                                      m_rearRight.getPosition()}, 
                                    getPose());



  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
//-  
    resetOdometry(defaultPose);
    resetEncoders();

//-
    //Camera setup
    photonCamera = new PhotonCamera(VisionConstants.kCameraName);
    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

      // Create pose estimator
      photonPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.kRobotToCam);

      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    } catch (IOException e) {

      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;

    }

//+    
    //pause gyro reset 1 sec to avoid interferring with gyro calibration
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {

        }
    }).start();

  }




@Override
public void periodic() {
//-
  // Query the latest result from PhotonVision   
  var result = photonCamera.getLatestResult();

  boolean hasTargets = result.hasTargets();
//-
 // SmartDashboard.putBoolean("Has Targets", hasTargets);
    if (hasTargets){
      // Get a list of currently tracked targets.
      //List<PhotonTrackedTarget> targets = result.getTargets();

      // Get the current best target.
      PhotonTrackedTarget target = result.getBestTarget();
       /* 
      // Get information from target. 
      SmartDashboard.putNumber("April Tag #", target.getFiducialId());
      SmartDashboard.putNumber("Pose Ambiguity", target.getPoseAmbiguity());
      SmartDashboard.putNumber("Target Yaw", target.getYaw());
      SmartDashboard.putNumber("Target Pitch", target.getPitch());
      SmartDashboard.putNumber("Target Area",target.getArea());
      SmartDashboard.putNumber("Target Skew", target.getSkew());
      */
      int targetID = target.getFiducialId();

      switch (targetID) {

        case 1: //AprilTag1
          targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag1[1]),
                                         Units.inchesToMeters(FieldConstants.kAprilTag1[2]),
                                         new Rotation2d(FieldConstants.kAprilTag1[4])
                                          );
          break;

        case 2: //AprilTag2
          targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag2[1]),
                                         Units.inchesToMeters(FieldConstants.kAprilTag2[2]),
                                         new Rotation2d(FieldConstants.kAprilTag2[4])
                                         );
          break;

        case 3: //AprilTag3
          targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag3[1]),
                                         Units.inchesToMeters(FieldConstants.kAprilTag3[2]),
                                         new Rotation2d(FieldConstants.kAprilTag3[4])
                                          );
          break;

        case 4:  //AprilTag4 - substation
           targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag4[1]),
                                   Units.inchesToMeters(FieldConstants.kAprilTag4[2]),
                                   new Rotation2d(FieldConstants.kAprilTag4[4])
                                  );
          break;
          
        case 5:  //AprilTag5 - substation
          targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag5[1]),
                                  Units.inchesToMeters(FieldConstants.kAprilTag5[2]),
                                  new Rotation2d(FieldConstants.kAprilTag5[4])
                                 );
         break;

        case 6: //AprilTag6
          targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag6[1]),
                                         Units.inchesToMeters(FieldConstants.kAprilTag6[2]),
                                         new Rotation2d(FieldConstants.kAprilTag6[4])
                                         );
          break;

        case 7: //AprilTag7
          targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag7[1]),
                                         Units.inchesToMeters(FieldConstants.kAprilTag7[2]),
                                         new Rotation2d(FieldConstants.kAprilTag7[4])
                                          );
          break;

        case 8: //AprilTag8
          targetPose = new Pose2d(Units.inchesToMeters(FieldConstants.kAprilTag8[1]),
                                         Units.inchesToMeters(FieldConstants.kAprilTag8[2]),
                                         new Rotation2d(FieldConstants.kAprilTag8[4])
                                         );
          break;


        
      } 
    }
//+
  // Update the odometry in the periodic block
  updateOdometry();

//- 
  // Update sensor readings
  SmartDashboard.putNumber("Gyro Angle",m_gyro.getAngle());
  SmartDashboard.putNumber("Gyro Rate",m_gyro.getRate());

  SmartDashboard.putNumber("Robot X", m_odometry.getPoseMeters().getX());
  SmartDashboard.putNumber("Robot Y", m_odometry.getPoseMeters().getY());
  SmartDashboard.putNumber("Robot Rotation",
    m_odometry.getPoseMeters().getRotation().getDegrees());
  
  SmartDashboard.putString("Target Location", targetPose.getTranslation().toString());

}

//-
public boolean getFieldRelative(){
  return m_gyro.isConnected();
}


//+
/**
 * Returns the currently-estimated pose of the robot.
 *
 * @return The pose.
 */
public Pose2d getPose() {
  return m_odometry.getPoseMeters();
}


//+
/**
 * Resets the odometry to the specified pose.
 *
 * @param pose The pose to which to set the odometry.
 */
public void resetOdometry(Pose2d pose) {
  m_odometry.resetPosition(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose
  );
}



/**
 * Method to drive the robot using joystick info.
 *
 * @param xSpeed Speed of the robot in the x direction (forward).
 * @param ySpeed Speed of the robot in the y direction (sideways).
 * @param rot Angular rate of the robot.
 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
 */


/* 
public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
  var swerveModuleStates =
    PhysicalConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed*100, ySpeed*100, rot*100, m_gyro.getRotation2d())
          : new ChassisSpeeds(xSpeed*100, ySpeed*100, rot*100)
    );
    
  SwerveDriveKinematics.desaturateWheelSpeeds(
    swerveModuleStates, 
    PhysicalConstants.kMaxSpeedMetersPerSecond
  );
  
  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  */

  
/**
 * Sets the swerve ModuleStates.
 *
 * @param desiredStates The desired SwerveModule states.
 */
public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, 
      PhysicalConstants.kMaxSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(desiredStates[0]);
  m_frontRight.setDesiredState(desiredStates[1]);
  m_rearLeft.setDesiredState(desiredStates[2]);
  m_rearRight.setDesiredState(desiredStates[3]);
}

/** Resets the drive encoders to currently read a position of 0. */
public void resetEncoders() {
  m_frontLeft.resetEncoders();
  m_rearLeft.resetEncoders();
  m_frontRight.resetEncoders();
  m_rearRight.resetEncoders();
}

/** Zeroes the heading of the robot. */
public void zeroHeading() {
  m_gyro.reset();
}

/**
 * Returns the heading of the robot.
 *
 * @return the robot's heading in degrees, from -180 to 180
 */
public double getHeading() {
  return Math.IEEEremainder(m_gyro.getAngle(),360);
}

public Rotation2d getRotation2d(){
  return Rotation2d.fromDegrees(getHeading());
}

public Pose2d getTargetPose(){
  return targetPose;
}

/**
 * Returns the turn rate of the robot.
 *
 * @return The turn rate of the robot, in degrees per second
 */
public double getTurnRate() {
  return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
}


public void stopModules() {
  m_frontLeft.stop();
  m_rearLeft.stop();
  m_frontRight.stop();
  m_rearRight.stop();
}

public void wheelsIn() {
  m_frontLeft.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)));
  m_rearLeft.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)));
  m_frontRight.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)));
  m_rearRight.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)));
  this.stopModules();

}

/** Updates the field-relative position. */
public void updateOdometry() {
  m_odometry.update(m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()});

  SmartDashboard.putNumber("Robot Heading", getHeading());
  SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

/* 

  m_poseEstimator.update(
    m_gyro.getRotation2d(), 
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  });

  Optional<EstimatedRobotPose> result =
    getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_poseEstimator.addVisionMeasurement(
          camPose.estimatedPose.toPose2d(), 
          camPose.timestampSeconds);
          //m_fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {
      // move it way off the screen to make it disappear
      //m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }
      
    // do i need the simulations?
    //m_fieldSim.getObject("Actual Pos").setPose(m_drivetrainSimulator.getPose());
    //m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());

    */
  }

    
  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
   *     the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
  
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
  }

}

