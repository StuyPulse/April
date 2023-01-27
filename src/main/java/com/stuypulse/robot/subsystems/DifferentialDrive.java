package com.stuypulse.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.AprilTagData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class DifferentialDrive extends SubsystemBase {

    private final Camera camera;

    /** SENSORS **/
    private final AHRS gyro;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    /** ODOMETRY **/
    private final DifferentialDriveKinematics kinematics;

    private final DifferentialDrivePoseEstimator poseEstimator;

    private final Field2d field;
    private final Field2d cameraField;

    public DifferentialDrive(Camera camera) {

        this.camera = camera;

        gyro = new AHRS(SPI.Port.kMXP);

        leftEncoder = new Encoder(-1, -1);
        rightEncoder = new Encoder(-1, -1);

        kinematics = new DifferentialDriveKinematics(0.7);

        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getGyroAngle(), 0, 0, Settings.STARTING_POSE);
        
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Math.toRadians(5)));
        
        reset(Settings.STARTING_POSE);

        field = new Field2d();
        cameraField = new Field2d();

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Camera Field", cameraField);
    }

    public Field2d getField() {
        return field;
    }

    public Field2d getCameraField() {
        return cameraField;
    }

    public void reset(Pose2d pose) {
        poseEstimator.resetPosition(getGyroAngle(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    }

    /** GYRO API */

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    /** ODOMETRY API */


    private void updatePose() {
        Optional<AprilTagData> pose = camera.getPoseData();
        if (pose.isPresent()) {
            AprilTagData poseData = pose.get();
            poseEstimator.addVisionMeasurement(poseData.pose, Timer.getFPGATimestamp() - poseData.latency);
        }
        poseEstimator.update(getGyroAngle(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {

        updatePose();

        field.getObject("pose estimator").setPose(getPose());
        cameraField.getObject("vision data").setPose(camera.getPoseData().get().pose);

        SmartDashboard.putNumber("DifferentialDrive/Pose Estimator X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("DifferentialDrive/Pose Estimator Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("DifferentialDrive/Pose Estimator Angle (deg)", getAngle().getDegrees());

        SmartDashboard.putNumber("DifferentialDrive/Gyro Angle", getAngle().getDegrees());
    }
}