package frc.robot.customClass;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance.*;

public class FieldPositions {
    public static enum Side {
        //Red side coordinants are still referenced to BLUE coordinates and rotations
        BLUE_12(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(180)))),
        BLUE_2(new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(120)))),
        BLUE_4(new Pose2d(0, 0, new Rotation2d(Math.toRadians(60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(60)))),
        BLUE_6(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
        BLUE_8(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(-60)))),
        BLUE_10(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120)))),
        RED_12(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
        RED_2(new Pose2d(0, 0, new Rotation2d(Math.toRadians(60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(60)))),
        RED_4(new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(120)))),
        RED_6(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(180)))),
        RED_8(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120)))),
        RED_10(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(-60))));


        public final Pose2d left;
        public final Pose2d right;

        private Side(Pose2d left, Pose2d right) {
            this.left = left;
            this.right = right;
        }
    }

}
