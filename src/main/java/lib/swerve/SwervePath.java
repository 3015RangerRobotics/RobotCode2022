package lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

/**
 * Custom version of the wpilib trajectory class that is
 * made to work better with PathPlanner and swerve drive
 */
public class SwervePath {
    public static final double TIME_STEP = 0.01;
    private ArrayList<State> states;

    /**
     * Construct a new Swerve Path
     */
    public SwervePath() {
        this.states = new ArrayList<>();
    }

    /**
     * Get all of the states in the path
     *
     * @return An arraylist of all path states
     */
    public ArrayList<State> getStates() {
        return this.states;
    }

    /**
     * Get the number of states in the path
     *
     * @return The number of states
     */
    public int numStates() {
        return this.states.size();
    }

    /**
     * Get the total runtime of a path
     *
     * @return Total runtime in seconds
     */
    public double getRuntime() {
        return states.get(states.size() - 1).time;
    }

    /**
     * Get the initial state in the path
     *
     * @return First state in the path
     */
    public State getInitialState() {
        return this.states.get(0);
    }

    /**
     * Create a SwervePath object from a CSV file
     * Expected format is xPos, yPos, velocity, acceleration, heading (direction robot is moving), rotation
     *
     * @param filename The path file to load
     * @return The SwervePath object
     */
    public static SwervePath fromCSV(String filename) {
        SwervePath traj = new SwervePath();

        try (BufferedReader br = new BufferedReader(new FileReader(new File(Filesystem.getDeployDirectory(), "paths/" + filename + ".csv")))) {
            String line = "";
            while ((line = br.readLine()) != null) {
                String[] point = line.split(",");

                double pos = Double.parseDouble(point[0]);
                double vel = Double.parseDouble(point[1]);
                double acc = Double.parseDouble(point[2]);
                double heading = Double.parseDouble(point[3]);
                double rotation = Double.parseDouble(point[4]);

                traj.states.add(new State(pos, Rotation2d.fromDegrees(heading), vel, acc, Rotation2d.fromDegrees(rotation), (traj.numStates() + 1) * TIME_STEP));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return traj;
    }

    private static double lerp(double startVal, double endVal, double t) {
        return startVal + (endVal - startVal) * t;
    }

    private static Translation2d lerp(Translation2d startVal, Translation2d endVal, double t) {
        return startVal.plus((endVal.minus(startVal)).times(t));
    }

    private static Rotation2d lerp(Rotation2d startVal, Rotation2d endVal, double t) {
        return startVal.plus((endVal.minus(startVal)).times(t));
    }

    /**
     * Sample the path at a given point in time
     *
     * @param time The elapsed time to sample
     * @return State of the path at the given time
     */
    public State sample(double time) {
        if (time <= states.get(0).time) {
            return states.get(0);
        }

        if (time >= getRuntime()) {
            return states.get(states.size() - 1);
        }

        int low = 1;
        int high = states.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (states.get(mid).time < time) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        State sample = states.get(low);
        State prevSample = states.get(low - 1);

        if (Math.abs(sample.time - prevSample.time) < 1E-5) {
            return sample;
        }

        return prevSample.interpolate(sample, (time - prevSample.time) / (sample.time - prevSample.time));
    }

    public static class State {
        private final double pos;
        private final Rotation2d heading;
        private final double velocity;
        private final double acceleration;
        public Rotation2d rotation;
        private final double time;

        /**
         * Construct a State
         *
         * @param pos          Translation2d holding the robot's position on the field
         * @param heading      Rotation2d representing the direction of robot motion
         * @param velocity     Velocity of the robot
         * @param acceleration Acceleration of the robot
         * @param rotation     Rotation of the robot
         * @param time         Time this state represents
         */
        public State(double pos, Rotation2d heading, double velocity, double acceleration, Rotation2d rotation, double time) {
            this.pos = pos;
            this.heading = heading;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.rotation = rotation;
            this.time = time;
        }


        public double getPos() {
            return this.pos;
        }

        public double getVelocity() {
            return this.velocity;
        }

        public Rotation2d getRotation() {
            return this.rotation;
        }

        public Rotation2d getHeading(){
            return this.heading;
        }

        public double getTime() {
            return this.time;
        }

        State interpolate(State endVal, double t) {
            double newT = lerp(time, endVal.time, t);
            double deltaT = newT - time;

            if (deltaT < 0) {
                return endVal.interpolate(this, 1 - t);
            }

            double newV = velocity + (acceleration * deltaT);
            double newS = (velocity * deltaT) + (0.5 * acceleration * Math.pow(deltaT, 2));

            double interpolationFrac = newS / (pos - endVal.pos);

            return new State(
                    lerp(pos, endVal.pos, interpolationFrac),
                    lerp(heading, endVal.heading, interpolationFrac),
                    newV,
                    acceleration,
                    lerp(rotation, endVal.rotation, interpolationFrac),
                    newT
            );
        }
    }
}