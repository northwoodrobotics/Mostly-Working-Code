// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;


public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  public static final SpectrumXboxController m_controller = new SpectrumXboxController(0, 0.1, 0.1);
  private static ShuffleboardTab master = Shuffleboard.getTab("master");


 

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new TeleopDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.leftStick.getY()) * DriveConstants.MAX_FWD_REV_SPEED_MPS,
            () -> -modifyAxis(m_controller.leftStick.getX()) * DriveConstants.MAX_FWD_REV_SPEED_MPS,
            () -> -modifyAxis(m_controller.rightStick.getX()) * DriveConstants.MAX_FWD_REV_SPEED_MPS
    ));

    // Configure the button bindings
    configureButtonBindings();
    ShowInputs();

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    m_controller.selectButton
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
  public void ShowInputs(){
    master.addNumber("X Input", ()-> Units.metersToFeet(-modifyAxis(m_controller.leftStick.getX()) * DriveConstants.MAX_FWD_REV_SPEED_MPS));
    master.addNumber("Y Input", () -> Units.metersToFeet(-modifyAxis(m_controller.leftStick.getY()) * DriveConstants.MAX_FWD_REV_SPEED_MPS));
    master.addNumber("Rotation Input", ()->Units.radiansToDegrees(-modifyAxis(m_controller.rightStick.getX())* DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC));
  }
}
