package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
        //This is a magic number from gridlock, may need to be changed or removed entirely
        public static final double PROCESS_LATENCY = 0.0472;
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
    }
