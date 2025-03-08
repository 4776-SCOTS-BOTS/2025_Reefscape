package frc.robot.customClass;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance.*;

public class SystemPositions {
    public static enum Positions {
        
        SAFE_STATION(Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters), 0.64),
        INTAKE_STATION(Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters), 0.69),
        L4_READY(1.708, 0.36),
        L3_READY(1.05, 0.365),
        L2_READY(Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters), 0.32),
        ARM_SAFE_HIGH(Constants.ElevatorConstants.ELEVATOR_BASE_HEIGHT.in(Meters), 0.58);

        public final double elevatorHeight;
        public final double armPosition;

        private Positions(double elevatorHeight, double armPosition) {
            this.elevatorHeight = elevatorHeight;
            this.armPosition = armPosition;
        }
    }

}
