package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotSubsystem { // Extend your base subsystem as needed

    // Create an instance of the DutyCycleEncoder on the appropriate DIO channel
    private final DutyCycleEncoder pivotEncoder;

    // An offset (in degrees) to align the physical zero of your mechanism.
    // Adjust this constant as needed.
    private static final double ABS_ENCODER_OFFSET = 0; 

    public PivotSubsystem() {
        // Initialize the encoder on, for example, DIO channel 0.
        pivotEncoder = new DutyCycleEncoder(0);
        // Optionally, if your encoder is mounted with a gear ratio (i.e. one encoder revolution
        // does not equal one full rotation of your mechanism), you can apply that conversion here.
        // For example:
        // pivotEncoder.setDistancePerPulse(360.0 * Constants.PivotConstants.GEAR_RATIO);
        // In our case, we assume the encoder directly measures the mechanism's rotation.
    }

    /**
     * Returns the current pivot angle in degrees.
     * The REV Through Bore Encoder outputs a duty cycle value between 0 and 1,
     * representing one full rotation. This method converts that to degrees
     * and applies an offset if needed.
     *
     * @return the pivot angle in degrees
     */
    public double getAngle() {
        // Read the raw encoder value (0 to 1) and convert to degrees.
        double rawDegrees = pivotEncoder.get() * 360.0;
        // Adjust by any pre-determined offset (for example, if the encoder's physical zero doesn't match your mechanism's zero)
        return rawDegrees - ABS_ENCODER_OFFSET;
    }
}
