package Simulation.ControllerAPI;

import java.util.LinkedList;
import Simulation.Messages.Message;

class MessageHandler {
	private final LinkedList<Message> myMessages;
	
	MessageHandler() {
		myMessages = new LinkedList<Message>();
	}
	
	void addMessageToMyList(Message message) {
		myMessages.addFirst(message);
	}
	
	void clearMyMessages() {
		myMessages.clear();
	}
	
	LinkedList<Message> getMyMessages() {
		return myMessages;
	}
}
