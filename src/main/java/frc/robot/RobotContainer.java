// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here... 
  private final SwerveDrive driveBase = new SwerveDrive(3, 2 * Math.PI, "geared upright", Constants.kinematics, Constants.config); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DriverControllerPort);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    ConfigShuffleboard();

    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                       this::getYSpeed, 
                                                       this::getRotationSpeed, driveBase));

    driveBase.enableDebugMode();
   
  }

  public void disabledInit() {
      }
  
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }
  
  private void ConfigShuffleboard(){
    SmartDashboard.putNumber("Swerve/X", 0.0);
    SmartDashboard.putNumber("Swerve/Y", 0.0);
    SmartDashboard.putNumber("Swerve/Z", 0.0);

    SwerveDriveCommand swerveDrive = new SwerveDriveCommand(() -> { return SmartDashboard.getNumber("Swerve/X", 0.0); },
                                                            () -> { return SmartDashboard.getNumber("Swerve/Y", 0.0); },
                                                            () -> { return SmartDashboard.getNumber("Swerve/Z", 0.0); },
                                                            driveBase);

    SmartDashboard.putData("Swerve/Start", swerveDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //driveBase.brakeMode();
    return new RunCommand(() -> {Pose2d currentPose = driveBase.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d midPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(4.0, 0.0)), new Rotation2d());
      Pose2d midPos2 = new Pose2d(currentPose.getTranslation().plus(new Translation2d(3, -1.0)), new Rotation2d());
      Pose2d midPos3 = new Pose2d(currentPose.getTranslation().plus(new Translation2d(4, -1.0)), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, midPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          1.0, 2.0, 
          Units.degreesToRadians(180), Units.degreesToRadians(360)
        ),  
        new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(90)))
      );

      AutoBuilder.followPathWithEvents(path).schedule();}, driveBase);
    
    }
  double getXSpeed(){ 
    int pov = driver.getHID().getPOV();
    double finalX;

    if ( pov == 0 )
      finalX = -0.75;
    else if(pov == 180)
      finalX = 0.75;
    else if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY();
    
    return -finalX;
  }

  public double getYSpeed(){ 
    int pov = driver.getHID().getPOV();

    double finalY;
    if ( pov == 270 || pov == 315 || pov == 225)
      finalY = -.5;
    else if(pov == 90 || pov == 45 || pov == 135)
      finalY = 0.5;
    else if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX();
    
    return -finalY; 
  } 
  
  public double getRotationSpeed(){ 
    double finalRotation;

      finalRotation = driver.getRightX();

      if (Math.abs(finalRotation) < 0.1)
        finalRotation = 0.0;
    
    return finalRotation;
  }

}