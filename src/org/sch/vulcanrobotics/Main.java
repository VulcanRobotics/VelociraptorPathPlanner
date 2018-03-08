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
        
        double trackWidth = 22.0 / 12.0; //29.872 / 12.0; //(25.75/12.0);
        
        // left starting position, by portal
        double leftOriginX = 33.0 / 12.0, leftOriginY =  fieldYWidth - ((29.69 + 11) / 12.0);
        
        // center starting position, right side of exchange zone
        double centerOriginX = 33.0 / 12.0, centerOriginY = exchangeZoneBottomY - (trackWidth / 2.0);
        
        // right starting position, by portal
        double rightOriginX = 33.0 / 12.0, rightOriginY = (29.69 + 11) / 12.0;
               

		WaypointSequence leftStartLeftScaleWaypoints = new WaypointSequence(10);
        leftStartLeftScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
        leftStartLeftScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(9, -3, Math.toRadians(-45)));

        WaypointSequence rightStartRightScaleWaypoints = new WaypointSequence(10);
        rightStartRightScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
        rightStartRightScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(7, 0,0));
        rightStartRightScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(21.0,15,Math.toRadians(45)));

        WaypointSequence centerStartLeftSwitchWaypoints = new WaypointSequence(10);
        centerStartLeftSwitchWaypoints.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
        centerStartLeftSwitchWaypoints.addWaypoint(new WaypointSequence.Waypoint(8.0,4.5,0.0));
        
        // FalconPathPlanner assumes absolute x,y positions on graph, whereas cheesyPoofs assume positions
        // are relative to initial robot position (i.e. cheesy position (0,0) is robot start)
        // convert cheesy Waypoints into absolute coordinates for FalconPathPlanner graphing library
       	
		double startTime = System.currentTimeMillis();

        startTime = System.currentTimeMillis();

        TrajectoryGenerator.Config trajConfig = new TrajectoryGenerator.Config();
		trajConfig.dt = .1;			// the time in seconds between each generated segment
		trajConfig.max_acc = 14.0;		// maximum acceleration for the trajectory, ft/s
		trajConfig.max_jerk = 28.0;	// maximum jerk (derivative of acceleration), ft/s
		trajConfig.max_vel = 6.5;		// maximum velocity you want the robot to reach for this trajectory, ft/s

        Path leftStartLeftScalePath = PathGenerator.makePath(leftStartLeftScaleWaypoints, trajConfig,
                trackWidth, "Left Start Left Scale");
        Path rightStartRightScalePath = PathGenerator.makePath(rightStartRightScaleWaypoints, trajConfig, trackWidth, 
        					"Right Start Right Scale");

		trajConfig.max_acc = 10.0;		// maximum acceleration for the trajectory, ft/s
		trajConfig.max_jerk = 14.0;	// maximum jerk (derivative of acceleration), ft/s
		trajConfig.max_vel = 7.0;		// maximum velocity you want the robot to reach for this trajectory, ft/s

        Path centerStartLeftSwitchPath = PathGenerator.makePath(centerStartLeftSwitchWaypoints, trajConfig, trackWidth, 
        					"Center Start Left Switch"); 
        
        System.out.println("Path calculated in " + (System.currentTimeMillis() - startTime) + "ms");
        
/*        
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
*/

        // plot the paths;
        FalconLinePlot plot = null;
        plot = plotTrajectory(plot, leftStartLeftScalePath, leftStartLeftScaleWaypoints, leftOriginX, leftOriginY);
        plotTrajectory(plot, rightStartRightScalePath, rightStartRightScaleWaypoints, rightOriginX, rightOriginY);
        plotTrajectory(plot, centerStartLeftSwitchPath, centerStartLeftSwitchWaypoints, centerOriginX, centerOriginY);

        // add vertical field midline
        double fieldCenterX = 27.0;
        plot.addData(new double[][] {{fieldCenterX,0.0},{fieldCenterX,fieldYWidth}}, Color.BLACK);
        
        // add portals;  x-intercepts are 3.0ft;  y are 29.69in
        plot.addData(new double[][] {{0.0, 29.69 / 12.0}, {3.0, 0.0}}, Color.black);
        plot.addData(new double[][] {{0.0, fieldYWidth - (29.69 / 12.0)}, 
        										{3.0, fieldYWidth}}, Color.BLACK);
        
        // add autoline
        plot.addData(new double[][] {{10.0,0.0},{10.0,fieldYWidth}},Color.BLACK);

        // add Exchange Zone
        plot.addData(exchangeZoneLines, Color.BLUE);
        
        // add switch
        plot.addData(switchTopPlate, Color.RED);
        plot.addData(switchBottomPlate, Color.BLUE);
        plot.addData(switchFence, Color.BLACK);
        
        // add power cube zone
        plot.addData(powerCubeZone, Color.BLACK);
        
        // add scale plates
        plot.addData(scaleTopPlate, Color.BLUE);
        plot.addData(scaleBottomPlate, Color.RED);
                              
        plot.xGridOn();
		plot.yGridOn();
		plot.setXTic(0, fieldXLength, 1);
		plot.setYTic(0, fieldYWidth, 1);       
	}        


	public static FalconLinePlot plotTrajectory(FalconLinePlot plot, Path p, WaypointSequence ws, double robotOriginX, double robotOriginY) {
		// plot the generated trajectories and velocity/position profiles
        Trajectory leftWheelTraj = p.getLeftWheelTrajectory(),
        		   rightWheelTraj = p.getRightWheelTrajectory();
        
		double[][] leftData = new double[leftWheelTraj.getNumSegments()][2],
				   rightData = new double[rightWheelTraj.getNumSegments()][2],
				   leftVel = new double[leftWheelTraj.getNumSegments()][2],
				   rightVel = new double[rightWheelTraj.getNumSegments()][2],
				   leftPos = new double[leftWheelTraj.getNumSegments()][2],
				   rightPos = new double[rightWheelTraj.getNumSegments()][2];

		for (int i=0; i<leftData.length;i++) {
			Segment s = leftWheelTraj.getSegment(i);
			leftData[i][0] = s.x + robotOriginX;
			leftData[i][1] = s.y + robotOriginY;
			if (i==0) {
				leftVel[i][0] = s.dt;
				leftPos[i][0] = s.dt;
			} else {
				leftVel[i][0] = leftVel[i-1][0] + s.dt;
				leftPos[i][0] = leftPos[i-1][0] + s.dt;
			}
			leftVel[i][1] = s.vel;
			leftPos[i][1] = s.pos;

			s = rightWheelTraj.getSegment(i);
			rightData[i][0] = s.x + robotOriginX;
			rightData[i][1] = s.y + robotOriginY;
			if (i==0) {
				rightVel[i][0] = s.dt;
				rightPos[i][0] = s.dt;
			} else {
				rightVel[i][0] = rightVel[i-1][0] + s.dt;
				rightPos[i][0] = rightPos[i-1][0] + s.dt;
			}
			rightVel[i][1] = s.vel;
			rightPos[i][1] = s.pos;
		}
        
        
		// plot trajectory on field window if it exists, or create new one if not
		if (plot == null) {
			plot = new FalconLinePlot(leftData, Color.MAGENTA, Color.MAGENTA);
			plot.addData(rightData, Color.MAGENTA, Color.MAGENTA);
		} else {
			plot.addData(leftData, Color.MAGENTA, Color.MAGENTA);
			plot.addData(rightData, Color.magenta, Color.MAGENTA);
		}
		
        // plot Waypoints on field
        for (int i = 1; i < ws.getNumWaypoints(); i++) {
        	plot.addData(new double[][] {{ws.getWaypoint(i-1).x + robotOriginX, ws.getWaypoint(i-1).y + robotOriginY},
        											{ws.getWaypoint(i).x + robotOriginX, ws.getWaypoint(i).y + robotOriginY}},
        											 Color.BLACK, Color.BLACK);
        }
		
        // create a new window for the motor position & velocity curves
        FalconLinePlot motorPlot = new FalconLinePlot(leftVel, Color.RED, Color.RED);
		motorPlot.addData(rightVel, Color.GREEN, Color.GREEN);
		motorPlot.addData(leftPos, Color.PINK, Color.PINK);
		motorPlot.addData(rightPos, Color.CYAN, Color.CYAN);
		motorPlot.xGridOn();
		motorPlot.yGridOn();
		motorPlot.setTitle(p.getName() + " - Velocity & Position - " + rightVel.length + " pts\n" + "LVel = Red, RVel = Green, LPos = Pink, RPos = Cyan");
		motorPlot.setYLabel("Ft / Ft per sec");
		motorPlot.setXLabel("Time (sec)");
		return plot;
	}
        

}
