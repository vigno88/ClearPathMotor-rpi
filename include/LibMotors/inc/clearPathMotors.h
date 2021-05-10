//Required include files
#include <stdio.h>	
#include <string>
#include <iostream>
#include <stdlib.h>
#include "pubSysCls.h"	

#define TIMEOUT 10000

using namespace sFnd;

class CP_Motors {
	public:
		void initialize();
		int nodeCount();
		
		void homeNodes();
		void startMovePosNode(int indexNode, int numSteps, int targetIsAbsolute);
		void startMoveVelNode(int indexNode, int vel);
	       	bool isMoveDoneNode(int indexNode);
		int readPosNode(int indexNode); 
		int readPosCommandedNode(int indexNode);
		int readVelNode(int indexNode);
		
		void setAccelNode(int indexNode, int accel);
		void setVelNode(int indexNode, int vel);

		void stopNodeHard(int indexNode);
		void stopNodeDecel(int indexNode);
		void stopNodeClear(int indexNode);
	private:
		SysManager _manager;
		IPort* _port;
		INode** _nodes;
		int _nodeCount;
	
		void openPort();
		void getNodes();
		void configNodes();
};
