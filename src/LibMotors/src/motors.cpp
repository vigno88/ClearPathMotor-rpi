//Required include files
#include "clearPathMotors.h"	
#include "motors.h"

typedef void CMotors;

CMotors* motors;

extern "C" {

	void Initialize() {
		motors = new CP_Motors();
		((CP_Motors*)motors)->initialize();
	}

	void HomeNodes() {
		((CP_Motors*)motors)->homeNodes();
	}

	int NodeCount() {
		return ((CP_Motors*)motors)->nodeCount();
	}

	void StartMovePosNode(int indexNode, int numSteps, int isTargetAbsolute) {
		((CP_Motors*)motors)->startMovePosNode(indexNode, numSteps, isTargetAbsolute);
	}
	
	void StartMoveVelNode(int indexNode, int velocity) {
		((CP_Motors*)motors)->startMoveVelNode(indexNode, velocity);
	}

	int IsMoveDoneNode(int indexNode) {
		if(((CP_Motors*)motors)->isMoveDoneNode(indexNode)) {
		    return 1;
		}
		return 0;
	}
	int ReadPosNode(int indexNode) {
		return ((CP_Motors*)motors)->readPosNode(indexNode);
	}
	int ReadPosCommandedNode(int indexNode) {
		return ((CP_Motors*)motors)->readPosCommandedNode(indexNode);
	}
	int ReadVelNode(int indexNode) {
		return ((CP_Motors*)motors)->readVelNode(indexNode);
	}


	void SetAccelNode(int indexNode, int accel) {
		((CP_Motors*)motors)->setAccelNode(indexNode, accel);
	}

	void SetVelNode(int indexNode, int vel) {
		((CP_Motors*)motors)->setVelNode(indexNode, vel);
	}

	void StopNodeHard(int indexNode) {
		((CP_Motors*)motors)->stopNodeHard(indexNode);
	}
	void StopNodeDecel(int indexNode) {
		((CP_Motors*)motors)->stopNodeDecel(indexNode);
	}
	void StopNodeClear(int indexNode) {
		((CP_Motors*)motors)->stopNodeClear(indexNode);
	}
}
