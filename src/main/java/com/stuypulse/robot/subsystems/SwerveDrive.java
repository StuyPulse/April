package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class SwerveDrive extends SubsystemBase {

    private final Camera camera;

    /** MODULES **/
    private final SwerveModule[] modules;

    /** SENSORS **/
    private final AHRS gyro;

    /** ODOMETRY **/
    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field;
    private final Field2d cameraField;


    public SwerveDrive(Camera camera) {
        this.camera = camera;

        this.modules = new SwerveModule[] {
            new SwerveModule("Front Right", new Translation2d(), new SmartAngle("Front Left Offset", null)),
            new SwerveModule("Front Left", new Translation2d(), new SmartAngle("Front Left Offset", null)),
            new SwerveModule("Back Left", new Translation2d(), new SmartAngle("Front Left Offset", null)),
            new SwerveModule("Back Right", new Translation2d(), new SmartAngle("Front Left Offset", null))
        };

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(
                getModuleStream()
                        .map(x -> x.getLocation())
                        .toArray(Translation2d[]::new));

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getModulePositions(), new Pose2d());
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

    /** MODULE API **/

    public SwerveModule getModule(String id) {
        for (SwerveModule module : modules) {
            if (module.getId().equals(id))
                return module;
        }

        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    public Translation2d getVelocity() {
        var speeds = getChassisSpeeds();
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Stream<SwerveModule> getModuleStream() {
        return Arrays.stream(getModules());
    }

    public SwerveModuleState[] getModuleStates() {
        return getModuleStream().map(x -> x.getState()).toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return getModuleStream().map(x -> x.getModulePosition()).toArray(SwerveModulePosition[]::new);
    }

    public void reset(Pose2d pose) {
        poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    /** MODULE STATES API **/

    public void setStates(Vector2D velocity, double omega, boolean fieldRelative) {
        setStates(new ChassisSpeeds(velocity.y, -velocity.x, -omega), fieldRelative);
    }

    public void setStates(Vector2D velocity, double omega) {
        setStates(velocity, omega, true);
    }

    public void setStates(ChassisSpeeds robotSpeed, Boolean fieldRelative) {
        if (fieldRelative){
            robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeed, getAngle());
        }
        setStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    public void stop() {
        setStates(new ChassisSpeeds(), true);
    }

    public void setStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                    "Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        for (int i = 0; i < states.length; ++i) {
            modules[i].setTargetState(states[i]);
        }
    }

    /** GYRO API */

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    /** ODOMETRY API */

    private static final Pose2d kNoPose = new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    private Pose2d visionData = kNoPose;


    private void updatePose() {
        Optional<AprilTagData> pose = camera.getPoseData();
        if (pose.isPresent()) {
            AprilTagData poseData = pose.get();
            poseEstimator.addVisionMeasurement(poseData.pose, Timer.getFPGATimestamp() - poseData.latency);
        }
        poseEstimator.update(getGyroAngle(), getModulePositions());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {

        updatePose();

        field.getObject("pose estimator").setPose(getPose());
        cameraField.getObject("vision data").setPose(visionData);

        // TODO: log angular velocity and velocity vector
        SmartDashboard.putNumber("Swerve/Pose Estimator X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Pose Estimator Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Pose Estimator Angle (deg)", getAngle().getDegrees());

        SmartDashboard.putNumber("Swerve/Gyro Angle", getAngle().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * 0.02));
        // gyro.setAngleAdjustment(getPose().getRotation().getDegrees());
    }
}