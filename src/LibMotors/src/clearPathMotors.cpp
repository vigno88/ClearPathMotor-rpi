#include "clearPathMotors.h"	

void CP_Motors::initialize() {
	_nodeCount = 0;
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
		exit(-1);
	}
	_manager.PortsOpen(portCount);
	_port = &_manager.Ports(0);

	printf(" Port[%d]: state=%d, nodes=%d\n",
		       	_port->NetNumber(),
			_port->OpenState(),
		       	_port->NodeCount());
}

void CP_Motors::getNodes() {
	_nodeCount =  _port->NodeCount();
	_nodes = new INode*[_nodeCount];
	for (size_t iNode = 0; iNode < _nodeCount; iNode++) {
		// Create a shortcut reference for a node
		_nodes[iNode] = &_port->Nodes(iNode);

		_nodes[iNode]->EnableReq(false);	

		_manager.Delay(200);

		printf("   Node[%d]: type=%d\n", int(iNode),
			       	_nodes[iNode]->Info.NodeType());
		printf("            userID: %s\n", 
				_nodes[iNode]->Info.UserID.Value());
		printf("        FW version: %s\n", 
				_nodes[iNode]->Info.FirmwareVersion.Value());
		printf("          Serial #: %d\n",
			       	_nodes[iNode]->Info.SerialNumber.Value());
		printf("             Model: %s\n", 
				_nodes[iNode]->Info.Model.Value());

		// The following statements will attempt to enable the node.  First,
		// any shutdowns or NodeStops are cleared, finally the node is enabled
		_nodes[iNode]->Status.AlertsClear();				
		_nodes[iNode]->Motion.NodeStopClear();			
		_nodes[iNode]->EnableReq(true);  //Enable node 
		//At this point the node is enabled
		printf("Node \t%zi enabled\n", iNode);
		//define a timeout in case the node is unable to enable
		double timeout = _manager.TimeStampMsec() + TIMEOUT;
		// This will loop checking on the Real time values of the node's
		// Ready status
		while (!_nodes[iNode]->Motion.IsReady()) {
			if (_manager.TimeStampMsec() > timeout) {
				printf("Error: Timed out waiting for Node %d to enable\n",
						iNode);
				// waits for user to press a key
				exit(-2);
			}
		}
	}
}

void CP_Motors::configNodes() {
	for (size_t iNode = 0; iNode < _nodeCount; iNode++) {
		//Set the units for Acceleration to RPM/SEC
		_nodes[iNode]->AccUnit(INode::RPM_PER_SEC);		
		//Set the units for Velocity to RPM
		_nodes[iNode]->VelUnit(INode::RPM);						
		_nodes[iNode]->Motion.PosnMeasured.AutoRefresh(true);
		//Set Acceleration Limit (RPM/Sec)
		_nodes[iNode]->Motion.AccLimit = ACC_NODE_1;
		//Set Velocity Limit (RPM)
		_nodes[iNode]->Motion.VelLimit = VEL_NODE_1;	
		_nodes[iNode]->Motion.Adv.DecelLimit = 5000;
	}
}

void CP_Motors::homeNodes() {
    for (size_t iNode = 0; iNode < _nodeCount; iNode++) {
        homeNode(iNode);
    }
}

void CP_Motors::homeNode(int indexNode) {
	    INode &theNode = *_nodes[indexNode];
            if (theNode.Motion.Homing.HomingValid()) {
                if (theNode.Motion.Homing.WasHomed()) {
                    printf("Node %d has already been homed,"
                        "current position is:\t%8.0f\n",
                        indexNode,
                        theNode.Motion.PosnMeasured.Value());
                    printf("Rehoming Node... \n");
                } else {
                    printf("Node [%d] has not been homed.  Homing Node now...\n",
                        indexNode);
                }
                //Now we will home the Node
                theNode.Motion.Homing.Initiate();
                //define a timeout in case the node is unable to enable
                double timeout = _manager.TimeStampMsec() + TIMEOUT;
                // Basic mode - Poll until disabled
                while (!theNode.Motion.Homing.WasHomed()) {
                    if (_manager.TimeStampMsec() > timeout) {
                        printf("Node did not complete homing:  \n\t"
                            "-Ensure Homing settings have been defined through"
                            " ClearView. \n\t -Check for alerts/Shutdowns \n\t"
                            " -Ensure timeout is longer than the longest" 
                            "possible homing move.\n");
                        exit(-2);
                    }
                }
                printf("Node completed homing\n");
            } 
}

//CP_Motors::~CP_Motors() {
//	delete [] _nodes;
//}


void CP_Motors::startMoveNode(int indexNode, int numSteps) {
	printf("Send movement of %d steps to node %d\n", numSteps, indexNode);
	_nodes[indexNode]->Motion.MovePosnStart(numSteps, true);			 
}

bool CP_Motors::isMoveDoneNode(int indexNode) {
	return _nodes[indexNode]->Motion.MoveIsDone();
}


int CP_Motors::readPosNode(int indexNode) {
	return _nodes[indexNode]->Motion.PosnMeasured;
}

int CP_Motors::readPosCommandedNode(int indexNode) {
	return _nodes[indexNode]->Motion.PosnCommanded;
}

int CP_Motors::readVelNode(int indexNode) {
	return _nodes[indexNode]->Motion.VelMeasured;
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

void CP_Motors::stopNodeClear(int indexNode) {
	_nodes[indexNode]->Motion.NodeStopClear();
}
	
void CP_Motors::backAndForthSequence(int indexNode, int travelLength) {
	while(true) {
		startMoveNode(indexNode, travelLength);
		while(!isMoveDoneNode(indexNode)){}
		startMoveNode(indexNode, -travelLength);
		while(!isMoveDoneNode(indexNode)){}
	}
}
