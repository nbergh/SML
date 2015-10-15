package Simulation;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.util.Timer;
import java.util.TimerTask;


final class Start extends JFrame implements ActionListener {
	// Objects
	Map map;
	VehicleStates vehicleStates;
	Parameters params;
	ControllerTalker controllerTalker;
	
	Start() {
		// Check all parameters
		params = new Parameters();
		params.checkParamerets();

		// Create map
		map = new Map(this);
		
		// Create vehicles
		vehicleStates = new VehicleStates(this);
		
		// Create controllerTalker
		controllerTalker = new ControllerTalker(this);
		
		// Graphics setup		
		JButton b2 = new JButton("Reset"); 		
		b2.setActionCommand("Reset");
		b2.addActionListener(this);	
		
		JPanel buttonPanel = new JPanel(new FlowLayout());		
		buttonPanel.add(b2);

		setLayout(new BorderLayout(5, 5));		
	    add(buttonPanel,BorderLayout.NORTH);
	    add(map.getMapGraphics(),BorderLayout.CENTER);
		
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		setTitle("Merging simulator");
	    setSize(params.mapParameters.windowWidth, params.mapParameters.windowHeight);
	    setVisible(true);		
	    
	    // Set graphics updating loop
	    Timer timer = new Timer();
	    timer.schedule(new UpdateGraphics(), 0, 1000/params.mapParameters.mapUpdateFPS); 
	    
	    // Set controller updating loop
	    timer.schedule(new QueryControllerTalker(), 0, 1000/params.vehiceParameters.controllerUpdateFrequency);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		// Button actions
		if ("Reset".equals(e.getActionCommand())) {
			
			vehicleStates.setVehicleStartingStates();
			controllerTalker.resetAllPaths();
			controllerTalker.resetAllMergingControllers();
		}
	}
	
	private final class UpdateGraphics extends TimerTask {
		// Updates the position of the cars and repaint the map component 
		@Override
		public void run() {
			// Update graphics
			vehicleStates.updateStates();
			map.repaintGraphics();
		}		
	}
	
	private final class QueryControllerTalker extends TimerTask {
		@Override
		public void run() {
			// Update the speed of the vehicles by querying the controllerTalker
			controllerTalker.setNewControlSignals();
		}
		
	}
	
	public static void main(String[] args) {
		new Start();
	}
	
}
