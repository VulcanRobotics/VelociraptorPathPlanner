package org.team1218.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.trajectory.TrajectoryGenerator.Config;
import com.team254.lib.trajectory.WaypointSequence;

public class LinkedWaypointSequences {
	List<WaypointSequence> waypointSequences = new ArrayList<WaypointSequence>();
	List<Config> configs = new ArrayList<Config>();
	List <Double> criticalSpeeds = new ArrayList <Double>();
	
	public void addTrajectories(WaypointSequence waypointSequence, Config config,double endSpeed) {
		System.out.println("Added WS of length:" + waypointSequence.getNumWaypoints());
		waypointSequences.add(waypointSequence);
		configs.add(config);
		criticalSpeeds.add(endSpeed);
	}
}
