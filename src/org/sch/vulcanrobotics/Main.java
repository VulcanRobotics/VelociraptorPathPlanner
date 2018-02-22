package org.sch.vulcanrobotics;

import java.awt.Color;
import java.io.File;
import java.io.FileWriter;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.io.TextFileSerializer;
import com.team254.lib.trajectory.Trajectory.Segment;

import usfirst.frc.team2168.robot.FalconLinePlot;
import usfirst.frc.team2168.robot.FalconPathPlanner;

public class Main {
	
	/* Trajectory points start at line 104 below */
	
	public static void main(String args[]) {
		
		double fieldYWidth = 27.0;	// y-axis
        double fieldXLength = 32.0;	// x-axis

        // exchange is 3ft by 4ft (x by y) and bottom edge is 1ft above horizontal midline
        double exchangeZoneBottomY = (fieldYWidth / 2.0) + 1.0,
        	   exchangeZoneTopY = exchangeZoneBottomY + 4.0,
        	   exchangeZoneLines[][] = new double[][] {
        			{0.0, exchangeZoneBottomY}, {3.0, exchangeZoneBottomY},
        			{3.0, exchangeZoneBottomY}, {3.0, exchangeZoneTopY},
        			{3.0, exchangeZoneTopY}, {0.0, exchangeZoneTopY}
        		};
        
        // switch center is x-axis 14ft, y-axis horizontal midline
        // switch plates are 4ft by 3ft (x by y) and centered x-axis 14ft, 12ft between outer edges  
        // switch fence is 4ft 8in by 12ft 9.5in (x by y)   
        double switchCenterX = 14.0, switchCenterY = fieldYWidth / 2.0;
        double switchTopPlateOuterY = switchCenterY + 6.0, switchBottomPlateOuterY = switchCenterY - 6.0;
        double[][] switchTopPlate = new double[][] {
        	{switchCenterX - 2.0, switchTopPlateOuterY},	{switchCenterX + 2.0, switchTopPlateOuterY},
        	{switchCenterX - 2.0, switchTopPlateOuterY}, {switchCenterX - 2.0, switchTopPlateOuterY - 3.0},
        	{switchCenterX - 2.0, switchTopPlateOuterY - 3.0}, {switchCenterX + 2.0, switchTopPlateOuterY - 3.0},
        	{switchCenterX + 2.0, switchTopPlateOuterY - 3.0}, {switchCenterX + 2.0, switchTopPlateOuterY}
        };
        double[][] switchBottomPlate = new double[][] {
        	{switchCenterX - 2.0, switchBottomPlateOuterY}, {switchCenterX + 2.0, switchBottomPlateOuterY},
        	{switchCenterX - 2.0, switchBottomPlateOuterY}, {switchCenterX - 2.0, switchBottomPlateOuterY + 3.0},
        	{switchCenterX - 2.0, switchBottomPlateOuterY + 3.0}, {switchCenterX + 2.0, switchBottomPlateOuterY + 3.0},
        	{switchCenterX + 2.0, switchBottomPlateOuterY + 3.0}, {switchCenterX + 2.0, switchBottomPlateOuterY}	
        };
        
        double switchFenceHalfXDepth = (4.0 + (8.0 / 12.0)) / 2.0,  	// 4ft 8in
        	   switchFenceHalfYWidth = (12.0 + (9.5 / 12.0)) / 2.0,	// 12ft 9.5in
        	   switchFenceLeftX = switchCenterX - switchFenceHalfXDepth,
        	   switchFenceRightX = switchCenterX + switchFenceHalfXDepth,
        	   switchFenceTopY = switchCenterY + switchFenceHalfYWidth,
        	   switchFenceBottomY = switchCenterY - switchFenceHalfYWidth;
        double[][] switchFence = new double[][] {
        	{switchFenceLeftX, switchFenceTopY}, {switchFenceRightX, switchFenceTopY},
        	{switchFenceLeftX, switchFenceTopY}, {switchFenceLeftX, switchFenceBottomY},
        	{switchFenceLeftX, switchFenceBottomY}, {switchFenceRightX, switchFenceBottomY},
        	{switchFenceRightX, switchFenceBottomY}, {switchFenceRightX, switchFenceTopY}
        };
        
        
        // power cube zone
        // zone is 3ft 6in by 3ft 9in (x by y), straddling horizontal midline and extending out from
        // left edge of switch fence
        double powerCubeZoneXDepth = 3.5, powerCubeZoneYWidth = 3.0 + (9.0 / 12.0);
        double powerCubeZoneTopY = (fieldYWidth / 2.0) + (powerCubeZoneYWidth / 2.0),
        	   powerCubeZoneBottomY = powerCubeZoneTopY - powerCubeZoneYWidth,
        	   powerCubeZoneLeftX = switchFenceLeftX - powerCubeZoneXDepth,
        	   powerCubeZoneRightX = powerCubeZoneLeftX + powerCubeZoneXDepth;
        double[][] powerCubeZone = new double[][] { 
        			{powerCubeZoneLeftX, powerCubeZoneTopY},{powerCubeZoneRightX, powerCubeZoneTopY},
        			{powerCubeZoneLeftX, powerCubeZoneTopY},{powerCubeZoneLeftX, powerCubeZoneBottomY},
        			{powerCubeZoneLeftX, powerCubeZoneBottomY},{powerCubeZoneRightX, powerCubeZoneBottomY}
        	
        };
        
        // scale center is x-axis 27ft, y-axis horizontal midline
        // scale plates are 4ft by 3ft (x by y) and centered x-axis 27ft, 15ft between outer edges
        double scaleCenterX = 27.0, scaleCenterY = fieldYWidth / 2.0;
        double scaleTopPlateOuterY = scaleCenterY + 7.5, scaleBottomPlateOuterY = scaleCenterY - 7.5,
        	   scalePlateLeftX = scaleCenterX - 2.0, scalePlateRightX = scaleCenterX + 2.0,
        	   scaleTopPlateInnerY = scaleTopPlateOuterY - 3.0, scaleBottomPlateInnerY = scaleBottomPlateOuterY + 3.0;
        double[][] scaleTopPlate = new double[][] {
        	{scalePlateLeftX, scaleTopPlateOuterY},{scalePlateLeftX, scaleTopPlateInnerY},
        	{scalePlateLeftX, scaleTopPlateInnerY},{scalePlateRightX, scaleTopPlateInnerY},
        	{scalePlateRightX, scaleTopPlateInnerY}, {scalePlateRightX, scaleTopPlateOuterY},
        	{scalePlateRightX, scaleTopPlateOuterY}, {scalePlateLeftX, scaleTopPlateOuterY}
        };
        double[][] scaleBottomPlate = new double[][] {
        	{scalePlateLeftX, scaleBottomPlateOuterY},{scalePlateLeftX, scaleBottomPlateInnerY},
        	{scalePlateLeftX, scaleBottomPlateInnerY},{scalePlateRightX, scaleBottomPlateInnerY},
        	{scalePlateRightX, scaleBottomPlateInnerY}, {scalePlateRightX, scaleBottomPlateOuterY},
        	{scalePlateRightX, scaleBottomPlateOuterY}, {scalePlateLeftX, scaleBottomPlateOuterY}
        };
        
        
        
		TrajectoryGenerator.Config cheesyConfig = new TrajectoryGenerator.Config();
		cheesyConfig.dt = .1;			// the time in seconds between each generated segment
		cheesyConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		cheesyConfig.max_jerk = 30.0;	// maximum jerk (derivative of acceleration), ft/s
		cheesyConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s

		WaypointSequence p = new WaypointSequence(10);
        p.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
        //p.addWaypoint(new WaypointSequence.Waypoint(12.0, -1.0, 0.0));
        p.addWaypoint(new WaypointSequence.Waypoint(21.0, -2.5, Math.toRadians(-20.0)));

        // FalconPathPlanner assumes absolute x,y positions on graph, whereas cheesyPoofs assume positions
        // are relative to initial robot position (i.e. cheesy position (0,0) is robot start)
        // convert cheesy Waypoints into absolute coordinates for FalconPathPlanner graphing library
        
        double robotOriginX = 33.0 / 12.0, robotOriginY = fieldYWidth - ((29.69 + 11) / 12.0) ; //23.0 - 0.0875 - (52.0 / 12.0);

		double trackWidth = 22.0 / 12.0; //29.872 / 12.0; //(25.75/12.0);
		
		double startTime = System.currentTimeMillis();

        startTime = System.currentTimeMillis();
        Path cheesyPath = PathGenerator.makePath(p, cheesyConfig,
                trackWidth, "Left Peg");
        System.out.println("cheesyPath calculated in " + (System.currentTimeMillis() - startTime) + "ms");
        
        TextFileSerializer tfs = new TextFileSerializer();
        String traj = tfs.serialize(cheesyPath);

        try {
        	FileWriter f = new FileWriter(new File("leftPeg.txt"));
        	f.write(traj);
        	f.flush();
        	f.close();
        } catch (Exception e) {
        	e.printStackTrace();
        }     


        // plot the generated trajectories and velocity/position profiles
        Trajectory cheesyLeftTrajectory = cheesyPath.getLeftWheelTrajectory(),
        		   cheesyRightTrajectory = cheesyPath.getRightWheelTrajectory();
        
		double[][] cheesyLeftPath = new double[cheesyLeftTrajectory.getNumSegments()][2],
				   cheesyRightPath = new double[cheesyRightTrajectory.getNumSegments()][2],
				   cheesyLeftVelocity = new double[cheesyLeftTrajectory.getNumSegments()][2],
				   cheesyRightVelocity = new double[cheesyRightTrajectory.getNumSegments()][2],
				   cheesyLeftPos = new double[cheesyLeftTrajectory.getNumSegments()][2],
				   cheesyRightPos = new double[cheesyRightTrajectory.getNumSegments()][2];

		for (int i=0; i<cheesyLeftPath.length;i++) {
			Segment s = cheesyLeftTrajectory.getSegment(i);
			cheesyLeftPath[i][0] = s.x + robotOriginX;
			cheesyLeftPath[i][1] = s.y + robotOriginY;
			if (i==0) {
				cheesyLeftVelocity[i][0] = s.dt;
				cheesyLeftPos[i][0] = s.dt;
			} else {
				cheesyLeftVelocity[i][0] = cheesyLeftVelocity[i-1][0] + s.dt;
				cheesyLeftPos[i][0] = cheesyLeftPos[i-1][0] + s.dt;
			}
			cheesyLeftVelocity[i][1] = s.vel;
			cheesyLeftPos[i][1] = s.pos;
			System.out.print(i + ": LtPos: " + s.pos + " LtVel: " + s.vel + " LtRPM: " + 
					(s.vel * 2173.0) * 60.0 / 1024.0 / 4.0);
			s = cheesyRightTrajectory.getSegment(i);
			cheesyRightPath[i][0] = s.x + robotOriginX;
			cheesyRightPath[i][1] = s.y + robotOriginY;
			if (i==0) {
				cheesyRightVelocity[i][0] = s.dt;
				cheesyRightPos[i][0] = s.dt;
			} else {
				cheesyRightVelocity[i][0] = cheesyRightVelocity[i-1][0] + s.dt;
				cheesyRightPos[i][0] = cheesyRightPos[i-1][0] + s.dt;
			}
			cheesyRightVelocity[i][1] = s.vel;
			cheesyRightPos[i][1] = s.pos;
			System.out.println(" RtPos: " + s.pos + " RtVel: " + s.vel + " RtRPM: " +
					(s.vel * 2173.0) * 60.0 / 1024.0 / 4.0);
		}
        
        
		// plot trajectory
		FalconLinePlot cheesyFieldPlot = new FalconLinePlot(cheesyLeftPath, Color.MAGENTA, Color.MAGENTA);
        cheesyFieldPlot.addData(cheesyRightPath, Color.MAGENTA, Color.MAGENTA);

        // plot Waypoints
        for (int i = 1; i < p.getNumWaypoints(); i++) {
        	cheesyFieldPlot.addData(new double[][] {{p.getWaypoint(i-1).x + robotOriginX, p.getWaypoint(i-1).y + robotOriginY},
        											{p.getWaypoint(i).x + robotOriginX, p.getWaypoint(i).y + robotOriginY}},
        											 Color.BLACK, Color.BLACK);
        }
        
        // add vertical field midline
        double fieldCenterX = 27.0;
        cheesyFieldPlot.addData(new double[][] {{fieldCenterX,0.0},{fieldCenterX,fieldYWidth}}, Color.BLACK);
        
        // add portals;  x-intercepts are 3.0ft;  y are 29.69in
        cheesyFieldPlot.addData(new double[][] {{0.0, 29.69 / 12.0}, {3.0, 0.0}}, Color.black);
        cheesyFieldPlot.addData(new double[][] {{0.0, fieldYWidth - (29.69 / 12.0)}, 
        										{3.0, fieldYWidth}}, Color.BLACK);
        
        // add autoline
        cheesyFieldPlot.addData(new double[][] {{10.0,0.0},{10.0,fieldYWidth}},Color.BLACK);

        // add Exchange Zone
        cheesyFieldPlot.addData(exchangeZoneLines, Color.BLUE);
        
        // add switch
        cheesyFieldPlot.addData(switchTopPlate, Color.RED);
        cheesyFieldPlot.addData(switchBottomPlate, Color.BLUE);
        cheesyFieldPlot.addData(switchFence, Color.BLACK);
        
        // add power cube zone
        cheesyFieldPlot.addData(powerCubeZone, Color.BLACK);
        
        // add scale plates
        cheesyFieldPlot.addData(scaleTopPlate, Color.BLUE);
        cheesyFieldPlot.addData(scaleBottomPlate, Color.RED);
                              
        cheesyFieldPlot.xGridOn();
		cheesyFieldPlot.yGridOn();
		cheesyFieldPlot.setXTic(0, fieldXLength, 1);
		cheesyFieldPlot.setYTic(0, fieldYWidth, 1);

		FalconLinePlot cheesyMotorPlot = new FalconLinePlot(cheesyLeftVelocity, Color.RED, Color.RED);
		cheesyMotorPlot.addData(cheesyRightVelocity, Color.GREEN, Color.GREEN);
		cheesyMotorPlot.addData(cheesyLeftPos, Color.PINK, Color.PINK);
		cheesyMotorPlot.addData(cheesyRightPos, Color.CYAN, Color.CYAN);
		cheesyMotorPlot.xGridOn();
		cheesyMotorPlot.yGridOn();
		

		System.out.println(" cheesyPath points: " + cheesyPath.getLeftWheelTrajectory().getNumSegments());
	}

}
