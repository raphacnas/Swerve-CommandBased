// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Helper.Constants;
import frc.robot.Helper.SwerveWrapper;

public class RobotContainer {

  private final PS4Controller ps5 = new PS4Controller(Constants.PS5_ID);
  private final SwerveWrapper swerve = new SwerveWrapper();

  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveDrive(swerve, ps5));
  }
  

  // private void configureBindings() {}
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
