package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

    // module data

    private final String id;
    private final Translation2d location;

    private SwerveModuleState targetState;

    // turn

    private final DutyCycleEncoder absoluteEncoder;
    private final SmartAngle angleOffset;

    // drive
    private final Encoder driveEncoder;

    public SwerveModule(String id, Translation2d location,
            SmartAngle angleOffset) {

        // module data
        this.id = id;
        this.location = location;

        targetState = new SwerveModuleState();

        // turn

        absoluteEncoder = new DutyCycleEncoder(-1);
        this.angleOffset = angleOffset;

        // drive

        driveEncoder = new Encoder(-1, -1);
    }

    public String getId() {
        return id;
    }

    public Translation2d getLocation() {
        return location;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getRotation2d());
    }

    private double getSpeed() {
        return driveEncoder.getRate();
    }

    private double getDistance() {
        return driveEncoder.getRate();
    }

    private Rotation2d getAbsolutePosition() {
        return new Rotation2d(MathUtil.interpolate(-Math.PI, +Math.PI, absoluteEncoder.getAbsolutePosition()));
    }

    private Rotation2d getRotation2d() {
        return getAbsolutePosition().minus(angleOffset.getRotation2d());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getRotation2d());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getRotation2d());
    }

    public void reset() {
        driveEncoder.reset();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber(id + "/Angle", getRotation2d().getDegrees());
        SmartDashboard.putNumber(id + "/Absolute Angle", getAbsolutePosition().getDegrees());

        SmartDashboard.putNumber(id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Speed", getSpeed());

    }
}