// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Helper.Constants;
import frc.robot.Helper.SwerveWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {

  private final PS5Controller ps5 = new PS5Controller(Constants.PS5_ID);
  private final SwerveWrapper swerve = new SwerveWrapper();

  private final SendableChooser<Command> autoChooser;
  private Command bFront, rFront, blDrop, rlDrop;

  public RobotContainer() {

    /* ---- STARTING POSE ---- */
    Pose2d poseInicial = new Pose2d(6.5, 4.0, Rotation2d.fromDegrees(180));
    swerve.resetOdometry(poseInicial);

    /* ---- AUTONOMOUS CHOOSER ---- */
    autoChooser = AutoBuilder.buildAutoChooser("FRONT2BACK-A");
            SmartDashboard.putData("Auto Mode", autoChooser);
            FollowPathCommand.warmupCommand().schedule();

    paths();
    configureBindings();

    swerve.setDefaultCommand(new SwerveDrive(swerve, ps5));
  }

  public void paths() {
    try { 
      blDrop = AutoBuilder.followPath(PathPlannerPath.fromPathFile("TO-B-LDROP")); 
      rlDrop = AutoBuilder.followPath(PathPlannerPath.fromPathFile("TO-R-LDROP"));
      bFront = AutoBuilder.followPath(PathPlannerPath.fromPathFile("TO-BF"));
      rFront = AutoBuilder.followPath(PathPlannerPath.fromPathFile("TO-RF"));

    } catch (Exception e) {
      DriverStation.reportError("Erro carregando paths", e.getStackTrace());
    }
  }
  

  private void configureBindings() {
    new JoystickButton(ps5, PS5Controller.Button.kCreate.value).onTrue(rFront);
    new JoystickButton(ps5, PS5Controller.Button.kOptions.value).onTrue(rlDrop);
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
