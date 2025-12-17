package frc.robot.Helper;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveWrapper extends SubsystemBase {

    private final SwerveDrive swerve;

    public SwerveWrapper() {
        try {
                File yagslDir = new File(Filesystem.getDeployDirectory(),"yagsl");

                swerve = new SwerveParser(yagslDir).createSwerveDrive(Constants.MAX_SPEED);
            } catch (Exception e) {
                throw new RuntimeException("Erro carregando configuração YAGSL!", e);
            }
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
}
