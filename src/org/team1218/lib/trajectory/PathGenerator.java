package org.team1218.lib.trajectory;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;

public class PathGenerator extends com.team254.lib.trajectory.PathGenerator {
	
	public static Path makeLinkedPath(LinkedWaypointSequences path, double wheelbaseWidth, String name) {
		return new Path(name, makeLeftAndRightTrajectories(generateLinkedTrajectory(path),wheelbaseWidth));
	}
	
	public static Trajectory generateLinkedTrajectory(LinkedWaypointSequences path) {
		Trajectory traj = new Trajectory(0);
		for(int i = 0; i < path.waypointSequences.size(); i++) {
			double startSpeed;
			double endSpeed;
			if(i == 0) {
				startSpeed = 0;
				endSpeed = path.criticalSpeeds.get(0);
			}else {
				startSpeed = path.criticalSpeeds.get(i-1);
				endSpeed = path.criticalSpeeds.get(i);
			}
			System.out.println(i);
			traj.append(generateFromPath(path.waypointSequences.get(i),path.configs.get(i),startSpeed,endSpeed));
		}
		return traj;
		
	}
	
	public static Trajectory generateFromPath(	WaypointSequence path,
		          								TrajectoryGenerator.Config config,
		          								double startSpeed,
		          								double endSpeed) {
		    if (path.getNumWaypoints() < 2) {
		      return null;
		    }

		    // Compute the total length of the path by creating splines for each pair
		    // of waypoints.
		    Spline[] splines = new Spline[path.getNumWaypoints() - 1];
		    double[] spline_lengths = new double[splines.length];
		    double total_distance = 0;
		    for (int i = 0; i < splines.length; ++i) {
		      splines[i] = new Spline();
		      if (!Spline.reticulateSplines(path.getWaypoint(i),
		              path.getWaypoint(i + 1), splines[i], Spline.QuinticHermite)) {
		        return null;
		      }
		      spline_lengths[i] = splines[i].calculateLength();
		      total_distance += spline_lengths[i];
		    }

		    // Generate a smooth trajectory over the total distance.
		    Trajectory traj = TrajectoryGenerator.generate(config,
		            TrajectoryGenerator.AutomaticStrategy,startSpeed, path.getWaypoint(0).theta,
		            total_distance, endSpeed, path.getWaypoint(path.getNumWaypoints()-1).theta);

		    // Assign headings based on the splines.
		    int cur_spline = 0;
		    double cur_spline_start_pos = 0;
		    double length_of_splines_finished = 0;
		    for (int i = 0; i < traj.getNumSegments(); ++i) {
		      double cur_pos = traj.getSegment(i).pos;

		      boolean found_spline = false;
		      while (!found_spline) {
		        double cur_pos_relative = cur_pos - cur_spline_start_pos;
		        if (cur_pos_relative <= spline_lengths[cur_spline]) {
		          double percentage = splines[cur_spline].getPercentageForDistance(
		                  cur_pos_relative);
		          traj.getSegment(i).heading = splines[cur_spline].angleAt(percentage);
		          double[] coords = splines[cur_spline].getXandY(percentage);
		          traj.getSegment(i).x = coords[0];
		          traj.getSegment(i).y = coords[1];
		          found_spline = true;
		        } else if (cur_spline < splines.length - 1) {
		          length_of_splines_finished += spline_lengths[cur_spline];
		          cur_spline_start_pos = length_of_splines_finished;
		          ++cur_spline;
		        } else {
		          traj.getSegment(i).heading = splines[splines.length - 1].angleAt(1.0);
		          double[] coords = splines[splines.length - 1].getXandY(1.0);
		          traj.getSegment(i).x = coords[0];
		          traj.getSegment(i).y = coords[1];
		          found_spline = true;
		        }
		      }
		    }

		    return traj;
		  }
}
