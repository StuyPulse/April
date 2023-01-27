package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* todo: make ICamera, SimCamera */
public class Camera extends SubsystemBase {

	private double[] botposeData;

	private DoubleEntry latencyEntry;
	private DoubleArrayEntry botposeEntry;

	public Camera() {
		final String name = "limelight-back";
		final NetworkTable limelight = 
			NetworkTableInstance.getDefault().getTable(name);

		botposeData = new double[] {};

		latencyEntry = 
			limelight.getDoubleTopic("tl").getEntry(0.0);
		botposeEntry = 
			limelight.getDoubleArrayTopic("botpose").getEntry(botposeData);

		for (int port : Settings.Limelight.PORTS) {
            PortForwarder.add(port, name + ".local", port);
        }
	}

	public double getLatency() {
		final double limelightCaptureDelay = 11.0;
		return 
			Units.millisecondsToSeconds(latencyEntry.get() + limelightCaptureDelay);
	}

    public boolean hasRobotPose() {
        return botposeData != null && botposeData.length == 6;
    }

	public Pose2d getRobotPose() {
		return new Pose2d(botposeData[0], botposeData[1], Rotation2d.fromDegrees(botposeData[5]));
	}

	@Override
	public void periodic() {
		botposeData = botposeEntry.get();
	}

}