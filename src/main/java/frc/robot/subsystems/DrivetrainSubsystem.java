// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.ExternalLib.CitrusLib.CitrusConstants.SwerveConstants;
import frc.ExternalLib.JackInTheBotLib.kinematics.SwerveOdometry;
import frc.robot.Constants;

import java.util.ArrayList;

import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

     
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */

  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */

  public SwerveDriveOdometry m_odometry;
  public PathPlannerTrajectory path;

 
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
 public  PIDController XController = new PIDController(AutoConstants.TRAJECTORYXkP, AutoConstants.TRAJECTORYXkI, AutoConstants.TRAJECTORYXkD);
 public PIDController YController = new PIDController(AutoConstants.TRAJECTORYYkP, AutoConstants.TRAJECTORYYkI, AutoConstants.TRAJECTORYYkD);

 public ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.THETACONTROLLERkP, AutoConstants.THETACONTROLLERkI, AutoConstants.THETACONTROLLERkD, AutoConstants.THETACONTROLLERCONSTRAINTS);






  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  //private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  //swerve kinematics
  private SwerveDriveKinematics m_kinematics = Constants.DriveConstants.KINEMATICS;
  public SwerveModuleState[] states;
  //SwerveDrivePoseEstimator m_poseEstimator;

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  // postion tracking
  



  public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_odometry = new SwerveDriveOdometry(m_kinematics, m_navx.getRotation2d());
    
        // There are 4 methods you can call to create your swerve modules.
        // The method you use depends on what motors you are using.
        //
        // Mk3SwerveModuleHelper.createFalcon500(...)
        //   Your module has two Falcon 500s on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createNeo(...)
        //   Your module has two NEOs on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createFalcon500Neo(...)
        //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
        //
        // Mk3SwerveModuleHelper.createNeoFalcon500(...)
        //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
        //
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.
    
        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        // FIXME Setup motor configuration
        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4SwerveModuleHelper.GearRatio.L2,
                // This is the ID of the drive motor
                DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET
        );
    
        // We will do the same for the other modules
        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET
        );
    
        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET
        );
    
        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET
        );
      }





    

   
  

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    //m_pigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
    //return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    // FIXME Uncomment if you are using a NavX
    if (m_navx.isMagnetometerCalibrated()) {
//      // //We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(360-m_navx.getFusedHeading());
   }
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
        this.states = desiredStates;
    }
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
          this.states = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

  
 
        
       
}

  public Pose2d getPose(){
          return m_odometry.getPoseMeters();
  }
  public Command createPathFollowCommand(PathPlannerTrajectory path, DrivetrainSubsystem subsystem){        
        
        PPSwerveControllerCommand swerveControllerCommand =
        new PPSwerveControllerCommand(
            path,
            ()->getPose(), // Functional interface to feed supplier
            Constants.DriveConstants.KINEMATICS,

            // Position controllers
            XController,
            YController,
            thetaController,
            commandStates -> this.states = commandStates,
            subsystem);
    return swerveControllerCommand;
 
     
    }




  @Override
  public void periodic() {
  

    
    
  }







}
