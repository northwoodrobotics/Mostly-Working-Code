package frc.robot.commands;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;




public class DriveFollowPath extends CommandBase{
    DrivetrainSubsystem drivetrainSubsystem;
    PathPlannerTrajectory trajectory; 
    public SwerveModuleState[] Swervestates;

    public DriveFollowPath(PathPlannerTrajectory path, DrivetrainSubsystem drivetrainSubsystem){
        this.trajectory = path;
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize(){




    }
    @Override
    public void execute(){
        drivetrainSubsystem.createPathFollowCommand(trajectory, drivetrainSubsystem);
    }

    @Override 
    public void end(boolean interrupted){
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }


}










   
