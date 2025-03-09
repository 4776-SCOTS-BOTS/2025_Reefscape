package frc.robot.customClass;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance.*;

public class FieldPositions {
    public static enum Loc {
        //Red side coordinants are still referenced to BLUE coordinates and rotations
        BLUE_1(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),1),
        BLUE_2(new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))),1),
        BLUE_3(new Pose2d(0, 0, new Rotation2d(Math.toRadians(60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(60))),1),
        BLUE_4(new Pose2d(3.275, 4.200, new Rotation2d(Math.toRadians(0))), new Pose2d(3.275, 3.833, new Rotation2d(Math.toRadians(0))),18),
        BLUE_5(new Pose2d(4.048, 5.191, new Rotation2d(Math.toRadians(-60))), new Pose2d(3.695, 5.058, new Rotation2d(Math.toRadians(-60))),19),
        BLUE_6(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))),1),
        RED_1(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),1),
        RED_2(new Pose2d(0, 0, new Rotation2d(Math.toRadians(60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(60))),1),
        RED_3(new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))),1),
        RED_4(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),1),
        RED_5(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))),1),
        RED_6(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(-60))),1);


        public final Pose2d left;
        public final Pose2d right;
        public final int tag;

        private Loc(Pose2d left, Pose2d right, int tag) {
            this.left = left;
            this.right = right;
            this.tag = tag;
        }

        public static Loc findByTag(int tag) {
            for (Loc loc : values()) {
                if (loc.tag == tag) {
                    return loc;
                }
            }
            return null;
        }
    }

    public static enum Side {
        LEFT,
        RIGHT;
    }

    public static Pose2d getTagCoord(int tag, Side side) {
        Loc loc = Loc.findByTag(tag);

        if (loc != null && side == Side.LEFT) {
            return loc.left;
        } else if (loc != null && side == Side.RIGHT) {
            return loc.right;
        } else {
            return null;
        }
    }

}
