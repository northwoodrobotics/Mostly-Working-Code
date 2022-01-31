package frc.robot.commands;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;




public class DriveFollowPath extends CommandBase{
    DrivetrainSubsystem drivetrainSubsystem;
    PathPlannerTrajectory path;

    private PIDController XController = new PIDController(AutoConstants.TRAJECTORYXkP, AutoConstants.TRAJECTORYXkI, AutoConstants.TRAJECTORYXkD);
    private PIDController YController = new PIDController(AutoConstants.TRAJECTORYYkP, AutoConstants.TRAJECTORYYkI, AutoConstants.TRAJECTORYYkD);

    private ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.THETACONTROLLERkP, AutoConstants.THETACONTROLLERkI, AutoConstants.THETACONTROLLERkD, AutoConstants.THETACONTROLLERCONSTRAINTS);



    public DriveFollowPath(PathPlannerTrajectory trajectory){
        this.path = trajectory;
        addRequirements(drivetrainSubsystem);
        

    }


    public void initialize(){

    }

    public void execute(){
        new PPSwerveControllerCommand(
            path, drivetrainSubsystem.getPose(), Constants.DriveConstants.KINEMATICS, XController, YController, thetaController, 
            drivetrainSubsystem., 
            requirements)
    }
}
