package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helper.Constants;
import frc.robot.Helper.SwerveWrapper;

public class SwerveDrive extends Command {

    private SwerveWrapper swerve;
    private PS5Controller ps5;

    double x, y, x2, y2, maxSpeed = Constants.MAX_SPEED;

  public SwerveDrive(SwerveWrapper SwerveW, PS5Controller PS5) {
    this.swerve = SwerveW;
    this.ps5 = PS5;

    addRequirements(SwerveW);
  }

  @Override
  public void initialize() {
    Pose2d poseInicial = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0));
    swerve.resetOdometry(poseInicial);
  }

  @Override
  public void execute() {
    x = ps5.getLeftX();  
    y = -ps5.getLeftY();  
    x2 = ps5.getRightX(); 

    // deadband
    if (Math.abs(y) < 0.05) y = 0;
    if (Math.abs(x)  < 0.05) x  = 0;
    if (Math.abs(x2)  < 0.05) x2  = 0;

    swerve.drive(new Translation2d(x * maxSpeed, y * maxSpeed), x2 * maxSpeed,true, true);
    SmartdashBoard();
  }

  public void SmartdashBoard() {
    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
