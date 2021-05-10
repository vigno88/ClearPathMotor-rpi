//Required include files
#include <stdio.h>	
#include <string>
#include <iostream>
#include <stdlib.h>
#include "pubSysCls.h"	

using namespace sFnd;

// Send message and wait for newline
void msgUser(const char *msg) {
	std::cout << msg;
	getchar();
}


class CP_Motors {
	public:
		void initialize();
		int nodeCount();

		void startMovePosNode(int indexNode, int numSteps);
		void startMoveVelNode(int indexNode, int velocity);
	       	bool isMoveDoneNode(int indexNode);
		int readPosNode(int indexNode); 
		
		void setAccelNode(int indexNode, int accel);
		void setVelNode(int indexNode, int vel);

		void backAndForthSequence(int indexNode, int travelLength);
		void stopNodeHard(int indexNode);
		void stopNodeDecel(int indexNode);

		~CP_Motors();
		SysManager _manager;
		IPort* _port;
		INode** _nodes;
		int _nodeCount = 0;
	private:
	/*	SysManager _manager;
		IPort* _port;
		INode** _nodes;
		int _nodeCount = 0;
	*/	
		void openPort();
		void getNodes();
		void configNodes();
		void homeNodes();

};

//*****************************************************************************
// This program will load configuration files onto each node connected to the 
// port, then executes sequential repeated moves on each axis.
//*****************************************************************************

#define ACC_LIM_NODE_1	400
#define VEL_SHORT_NODE_1	300
#define VEL_LONG_NODE_1	300
#define VEL_HOMING_NODE_1 100

#define DWELL_SHORT			1
#define DWELL_LONG			1

#define MOVE_DISTANCE_NODE_1	7000

#define NUM_MOVES			1

#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)

int main(int argc, char* argv[]) {
	msgUser("Motion Example starting. Press Enter to continue.");
	CP_Motors motors;

	try { 
		motors.initialize();

		///////////////////////////////////////////////////////////////////////
		//At this point we will execute 10 rev moves sequentially on each axis
		///////////////////////////////////////////////////////////////////////
		INode &Node1 = *motors._nodes[0];
		motors.setVelNode(0, 4100);
		motors.setAccelNode(0, 12000);
		motors.setVelNode(2, 4100);
		motors.setAccelNode(2, 12000);
		motors.setVelNode(4, 4100);
		motors.setAccelNode(4, 12000);
		motors.setVelNode(1, 400);
		motors.setAccelNode(1, 1000);
		motors.setVelNode(3, 400);
		motors.setAccelNode(3, 1000);
		motors.setVelNode(5, 400);
		motors.setAccelNode(5, 1000);
		motors.backAndForthSequence(0, 500);
//		for(size_t j=0; j < 10;j++) {
//			for (size_t i = 0; i < NUM_MOVES; i++) {
/*				printf("Moving Nodes...Current Positions: \n");
				Node1.Motion.VelLimit = VEL_HOMING_NODE_1;	
				//motors.startMoveNode(0,-MOVE_DISTANCE_NODE_1);
				printf("Start Move of %d step", 0);
				motors.startMoveNode(0,-7000);
				Node1.Motion.MovePosnStart(MOVE_DISTANCE_NODE_1);			 
				//define a timeout in case the node is unable to enable
				double timeout = motors._manager.TimeStampMsec() + 5000;			
				//while (!Node1.Motion.MoveIsDone()) {
				while (!motors.isMoveDoneNode(0)) {
					if (motors._manager.TimeStampMsec() > timeout) {
						printf("Error: Timed out waiting for move to complete\n");
						msgUser("Press any key to continue."); 
						return -2;
				}
					//Node1.CPMStatus
			//		printf("Node 0 pos: %.1f\n", (double)Node1.Motion.PosnMeasured);
			//		printf("Node 0 torque: %.1f\n", (double)Node1.Motion.TrqMeasured);
				}
				printf("Node 0: %.1f\n", (double)Node1.Motion.PosnMeasured);

				printf("Move completed\n");
				motors._manager.Delay(DWELL_SHORT); */
//			} // for each move

/*			motors._manager.Delay(DWELL_SHORT);
			//Set Velocity Limit (RPM)
			Node1.Motion.VelLimit = VEL_LONG_NODE_1;
			printf("Returning to original position...");
			Node1.Motion.MovePosnStart(0, true); 
			double timeout = motors._manager.TimeStampMsec() + 5000;
			while (!Node1.Motion.MoveIsDone()) {
				if (motors._manager.TimeStampMsec() > timeout) {
					printf("Error: Timed out waiting for move to complete\n");
					msgUser("Press any key to continue."); 
					return -2;
				}
				printf("Node 0: %.1f\r", (double)Node1.Motion.PosnMeasured);
			}
			printf("Node 0: %.1f\n", (double)Node1.Motion.PosnMeasured);

			printf("Move completed\n");
			motors._manager.Delay(DWELL_LONG); */
//		}


		///////////////////////////////////////////////////////////////////////
		//After moves have completed Disable node, and close ports
		///////////////////////////////////////////////////////////////////////
		printf("Disabling nodes, and closing port\n");
		//Disable Nodes
		for (size_t iNode = 0; iNode < motors._port->NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			motors._port->Nodes(iNode).EnableReq(false);
		}
	} catch (mnErr& theErr) {
		// This statement will print the address of the error, the error code
		// (defined by the mnErr class),as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n",
				theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		msgUser("Press any key to continue.");
		return 0;  
	}

	// Close down the ports
	motors._manager.PortsClose();
	msgUser("Press any key to continue.");
	return 0;	
}

void CP_Motors::initialize() {
	_manager = SysManager();
	openPort();
	getNodes();
	configNodes();
	homeNodes();
}

int CP_Motors::nodeCount() {
	return _nodeCount;
}

void CP_Motors::openPort() {
	size_t portCount = 0;

	std::vector<std::string> comHubPorts;
	SysManager::FindComHubPorts(comHubPorts);
	printf("Found %d SC Hubs\n", comHubPorts.size());
	for (portCount = 0;
			portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX;
			portCount++) {
		// Define the first SC Hub port (port 0) to be associated with COM
		// portnum (as seen in device manager)
		_manager.ComHubPort(portCount, comHubPorts[portCount].c_str()); 	
	}
	if (portCount < 0) {
		printf("Unable to locate SC hub port\n");
		// waits for user to press a key
		msgUser("Press any key to continue."); 
		exit(-1);
	}
	_manager.PortsOpen(portCount);
	_port = &_manager.Ports(0);

	printf(" Port[%d]: state=%d, nodes=%d\n", _port->NetNumber(),
			_port->OpenState(), _port->NodeCount());
}

void CP_Motors::getNodes() {
	_nodeCount =  _port->NodeCount();
	_nodes = new INode*[_nodeCount];
	for (size_t iNode = 0; iNode < _nodeCount; iNode++) {
		// Create a shortcut reference for a node
		_nodes[iNode] = &_port->Nodes(iNode);

		_nodes[iNode]->EnableReq(false);	

		_manager.Delay(200);

		printf("   Node[%d]: type=%d\n", int(iNode), _nodes[iNode]->Info.NodeType());
		printf("            userID: %s\n", _nodes[iNode]->Info.UserID.Value());
		printf("        FW version: %s\n", 
				_nodes[iNode]->Info.FirmwareVersion.Value());
		printf("          Serial #: %d\n", _nodes[iNode]->Info.SerialNumber.Value());
		printf("             Model: %s\n", _nodes[iNode]->Info.Model.Value());

		// The following statements will attempt to enable the node.  First,
		// any shutdowns or NodeStops are cleared, finally the node is enabled
		_nodes[iNode]->Status.AlertsClear();				
		_nodes[iNode]->Motion.NodeStopClear();  				
		_nodes[iNode]->EnableReq(true);  //Enable node 
		//At this point the node is enabled
		printf("Node \t%zi enabled\n", iNode);
		//define a timeout in case the node is unable to enable
		double timeout = _manager.TimeStampMsec() + TIME_TILL_TIMEOUT;
		// This will loop checking on the Real time values of the node's
		// Ready status
		while (!_nodes[iNode]->Motion.IsReady()) {
			if (_manager.TimeStampMsec() > timeout) {
				printf("Error: Timed out waiting for Node %d to enable\n",
						iNode);
				// waits for user to press a key
				msgUser("Press any key to continue.");
				exit(-2);
			}
		}
	}
}

void CP_Motors::configNodes() {
	printf("node count: %d", _nodeCount);
	for (size_t iNode = 0; iNode < _nodeCount; iNode++) {
		//Set the units for Acceleration to RPM/SEC
		_nodes[iNode]->AccUnit(INode::RPM_PER_SEC);		
		//Set the units for Velocity to RPM
		_nodes[iNode]->VelUnit(INode::RPM);						
		_nodes[iNode]->Motion.PosnMeasured.AutoRefresh(true);
		//Set Acceleration Limit (RPM/Sec)
		_nodes[iNode]->Motion.AccLimit = ACC_LIM_NODE_1;
		//Set Velocity Limit (RPM)
		_nodes[iNode]->Motion.VelLimit = VEL_SHORT_NODE_1;	
	}
}

void CP_Motors::homeNodes() {
	for (size_t iNode = 0; iNode < _nodeCount; iNode++) {
	    INode &theNode = *_nodes[iNode];
            if (theNode.Motion.Homing.HomingValid()) {
                if (theNode.Motion.Homing.WasHomed()) {
                    printf("Node %d has already been homed,"
                        "current position is:\t%8.0f\n",
                        iNode,
                        theNode.Motion.PosnMeasured.Value());
                    printf("Rehoming Node... \n");
                } else {
                    printf("Node [%d] has not been homed.  Homing Node now...\n",
                        iNode);
                }
                //Now we will home the Node
                theNode.Motion.Homing.Initiate();
            } 
	}
	for (size_t iNode = 0; iNode < _nodeCount; iNode++) {
	    INode &theNode = *_nodes[iNode];
            while(theNode.Motion.Homing.IsHoming()) {}
	    printf("Homing done");
	}
}

CP_Motors::~CP_Motors() {
	delete [] _nodes;
}


void CP_Motors::startMovePosNode(int indexNode, int numSteps) {
	_nodes[indexNode]->Motion.MovePosnStart(numSteps, true);			 
}

void CP_Motors::startMoveVelNode(int indexNode, int velocity) {
	_nodes[indexNode]->Motion.MoveVelStart(velocity);			 
}

bool CP_Motors::isMoveDoneNode(int indexNode) {
	return _nodes[indexNode]->Motion.MoveIsDone();
}


int CP_Motors::readPosNode(int indexNode) {
	return _nodes[indexNode]->Motion.PosnMeasured;
}


void CP_Motors::setAccelNode(int indexNode, int accel) {
	printf("Modified the acceleration of node %d to %d RPM/sec\n",  indexNode, accel);
	_nodes[indexNode]->Motion.AccLimit = accel;	
}


void CP_Motors::setVelNode(int indexNode, int vel) {
	printf("Modified the velocity of node %d to %d RPM\n",  indexNode, vel);
	_nodes[indexNode]->Motion.VelLimit = vel;	
}

void CP_Motors::stopNodeHard(int indexNode) {
	_nodes[indexNode]->Motion.NodeStop(STOP_TYPE_ABRUPT);
}

void CP_Motors::stopNodeDecel(int indexNode) {
	_nodes[indexNode]->Motion.NodeStop(STOP_TYPE_RAMP_AT_DECEL);
}
	
void CP_Motors::backAndForthSequence(int indexNode, int travelLength) {
	_nodes[1]->Motion.MoveVelStart(-400);			 
	_nodes[3]->Motion.MoveVelStart(-400);			 
	_nodes[5]->Motion.MoveVelStart(-400);			 
		int travel = travelLength;
			startMovePosNode(0, travel);
			startMovePosNode(2, -travel);
			startMovePosNode(4, travel);
	while(true) {
		if(isMoveDoneNode(0)) {
			travel = -travel;
			startMovePosNode(0, travel);
			startMovePosNode(2, -travel);
			startMovePosNode(4, travel);

		}
		
		}
	}
