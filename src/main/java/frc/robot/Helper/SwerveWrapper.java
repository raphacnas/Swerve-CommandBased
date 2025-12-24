package frc.robot.Helper;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveWrapper extends SubsystemBase {

    private final SwerveDrive swerve;

    /* ----- NOVO: publishers do NT4 ----- */
    private final StructArrayPublisher<SwerveModuleState> desiredPub;
    private final StructPublisher<Rotation2d> rotationPub;

    public SwerveWrapper() {
        try {
                File yagslDir = new File(Filesystem.getDeployDirectory(),"yagsl");

                swerve = new SwerveParser(yagslDir).createSwerveDrive(Constants.MAX_SPEED);
                 RobotConfig config = RobotConfig.fromGUISettings();

                /* ---- AutoBuilder ---- */
                AutoBuilder.configure(
                    swerve::getPose,
                    swerve::resetOdometry,
                    swerve::getRobotVelocity,
                    (speeds, feedforwards) -> swerve.setChassisSpeeds(speeds),
                    new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), 
                        new PIDConstants(5.0, 0.0, 0.0)
                    ),
                    config,
                    () -> DriverStation.isAutonomous(), 
                    this
                );

            } catch (Exception e) {
                throw new RuntimeException("Erro carregando configuração YAGSL!", e);
            }


        /* ----- Simulador ----- */
        var nt = NetworkTableInstance.getDefault();
        desiredPub  = nt.getStructArrayTopic("/Swerve/desired",  SwerveModuleState.struct).publish();
        rotationPub = nt.getStructTopic("/Swerve/rotation", Rotation2d.struct).publish();
    }

    @Override
    public void periodic() {
        desiredPub.set(getModuleStates());
        rotationPub.set(swerve.getPose().getRotation());
    }

    public void drive(Translation2d translation, double rot, boolean fieldCentric, boolean isOpenLoop) {
        swerve.drive(translation, rot, fieldCentric, isOpenLoop);
    }

    public void zeroGyro() {
        swerve.zeroGyro();
    }

    public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        swerve.setModuleStates(states, isOpenLoop);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public SwerveModuleState[] getModuleStates() {
        return swerve.getStates();
    }

    public void resetOdometry(Pose2d pose) {
        swerve.resetOdometry(pose);
    }

    
    public ChassisSpeeds getChassisSpeeds() {
        return swerve.getRobotVelocity();
    }
}
