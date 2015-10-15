package Simulation.Messages;

public class SafeToMergeReplyMessage extends Message {
	public boolean safeToMerge=false;
	
	public SafeToMergeReplyMessage(int sender) {
		super(sender);
	}
	
	
}
