package frc.robot.sim;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

import box3dsim.math.Pose3;
import box3dsim.math.Quat;

/** box3d-sim ↔ WPILib geometry. */
final class PoseConversions {
    private PoseConversions() {}

    static Pose3d toPose3d(Pose3 p) {
        Quat q = p.rotation();
        return new Pose3d(new Translation3d(p.position().x(), p.position().y(), p.position().z()),
                new Rotation3d(new Quaternion(q.w(), q.x(), q.y(), q.z())));
    }

    static Pose2d toPose2d(Pose3 p) {
        return new Pose2d(p.position().x(), p.position().y(), new Rotation2d(p.rotation().yaw()));
    }

    static Pose3 fromPose2d(Pose2d p) {
        return Pose3.ofXYYaw(p.getX(), p.getY(), 0, p.getRotation().getRadians());
    }
}
