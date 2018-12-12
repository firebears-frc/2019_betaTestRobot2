package org.firebears.motion;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class TrajectoryFactory {

        public Trajectory trajectory1 = null;
        public Trajectory.Config config = null;

        public void init() {
                config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
                                Trajectory.Config.SAMPLES_HIGH, 
                                0.05, 1.7, 2.0, 60.0);
                Waypoint[] points = new Waypoint[] { 
                        new Waypoint(-4, -1, Pathfinder.d2r(-45)), 
                        new Waypoint(-2, -2, 0),
                        new Waypoint(0, 0, 0) };

                trajectory1 = Pathfinder.generate(points, config);

                File dir = new File("/U/");
                if (!dir.exists()) {
                        dir = new File("/home/lvuser/");
                }
                Pathfinder.writeToCSV(new File(dir, "trajectory1"), trajectory1);
        }
}
