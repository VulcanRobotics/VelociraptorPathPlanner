package org.team1218.lib.trajectory;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;

public class TurnGenerator {
	
	public static Path generateTurn(double angle,TrajectoryGenerator.Config config, double trackWidth) {
		
		double distance = (angle*trackWidth*0.5)/0.90;
		double k = Math.min(4*config.max_vel, Math.sqrt(distance*10*config.max_acc));
		double t = 2.944*(distance/k)*2;
		System.out.println(t);
		int length = (int)(t/config.dt);
		Trajectory lTraj = new Trajectory(length+2);
		Trajectory rTraj = new Trajectory(length+2);
		lTraj.getSegment(0).pos = 0;
		rTraj.getSegment(0).pos = 0;
		lTraj.getSegment(length+1).pos = distance*0.90;
		rTraj.getSegment(length+1).pos = -distance*0.90;
		
		lTraj.getSegment(0).dt = config.dt;
		rTraj.getSegment(0).dt = config.dt;
		lTraj.getSegment(length+1).dt = config.dt;
		rTraj.getSegment(length+1).dt = config.dt;
		
		for(int i = 0; i < length; i++) {
			double x = (((double)i/length)*t)-t/2;
			double d = distance/(1+Math.pow(Math.E, ((-k*x)/distance)));
			lTraj.getSegment(i+1).pos = d-distance*0.05;
			rTraj.getSegment(i+1).pos = -(d-distance*0.05);
			lTraj.getSegment(i+1).dt = config.dt;
			rTraj.getSegment(i+1).dt = config.dt;
		}
		
		for(int i = 0; i < length+1; i++) {
			lTraj.getSegment(i).vel = (lTraj.getSegment(i+1).pos - lTraj.getSegment(i).pos)/config.dt;
			rTraj.getSegment(i).vel = (rTraj.getSegment(i+1).pos - rTraj.getSegment(i).pos)/config.dt;
		}
		
		Trajectory.Pair pair = new Trajectory.Pair(lTraj, rTraj);
		Path path = new Path("turn",pair);
		return path;
	}
}
