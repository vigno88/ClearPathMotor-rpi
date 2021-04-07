//Required include files
#include <stdio.h>	
#include <string>
#include <iostream>
#include "pubSysCls.h"	

using namespace sFnd;

// Send message and wait for newline
void msgUser(const char *msg) {
    std::cout << msg;
    getchar();
}

//*****************************************************************************
// This program will load configuration files onto each node connected to the 
// port, then executes sequential repeated moves on each axis.
//*****************************************************************************

#define ACC_LIM_NODE_1	400
#define VEL_SHORT_NODE_1	300
#define VEL_LONG_NODE_1	300

#define DWELL_SHORT			1
#define DWELL_LONG			1

#define MOVE_DISTANCE_NODE_1	-10

#define NUM_MOVES			1

#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)

int main(int argc, char* argv[]) {
    msgUser("Motion Example starting. Press Enter to continue.");

    size_t portCount = 0;
    std::vector<std::string> comHubPorts;


    // Create the SysManager object. This object will coordinate actions among 
    // various ports and within nodes. In this example we use this object to setup
    // and open our port.
    SysManager myMgr;

    // This will try to open the port. If there is an error/exception during the
    // port opening, the code will jump to the catch loop where detailed
    // information regarding the error will be displayed; otherwise the catch loop
    // is skipped over
    try { 
        SysManager::FindComHubPorts(comHubPorts);
        printf("Found %d SC Hubs\n", comHubPorts.size());
        for (portCount = 0;
                portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX;
                portCount++) {
            // Define the first SC Hub port (port 0) to be associated with COM
            // portnum (as seen in device manager)
            myMgr.ComHubPort(portCount, comHubPorts[portCount].c_str()); 	
        }
        if (portCount < 0) {
            printf("Unable to locate SC hub port\n");
            // waits for user to press a key
            msgUser("Press any key to continue."); 
            return -1;
        }
        //Open the port
        myMgr.PortsOpen(portCount);
        IPort &myPort = myMgr.Ports(0);
        printf(" Port[%d]: state=%d, nodes=%d\n", myPort.NetNumber(),
                myPort.OpenState(), myPort.NodeCount());

        //Once the code gets past this point, it can be assumed that the Port has
        // been opened without issue. Now we can get a reference to our port object
        // which we will use to access the node objects
        for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
            // Create a shortcut reference for a node
            INode &theNode = myPort.Nodes(iNode);

            theNode.EnableReq(false);	

            myMgr.Delay(200);

            printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
            printf("            userID: %s\n", theNode.Info.UserID.Value());
            printf("        FW version: %s\n", 
                theNode.Info.FirmwareVersion.Value());
            printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
            printf("             Model: %s\n", theNode.Info.Model.Value());

            // The following statements will attempt to enable the node.  First,
            // any shutdowns or NodeStops are cleared, finally the node is enabled
            theNode.Status.AlertsClear();				
            theNode.Motion.NodeStopClear();  				
            theNode.EnableReq(true);  //Enable node 
            //At this point the node is enabled
            printf("Node \t%zi enabled\n", iNode);
            //define a timeout in case the node is unable to enable
            double timeout = myMgr.TimeStampMsec() + TIME_TILL_TIMEOUT;
            // This will loop checking on the Real time values of the node's
            // Ready status
            while (!theNode.Motion.IsReady()) {
                if (myMgr.TimeStampMsec() > timeout) {
                    printf("Error: Timed out waiting for Node %d to enable\n",
                        iNode);
                    // waits for user to press a key
                    msgUser("Press any key to continue.");
                    return -2;
                }
            }
            // At this point the Node is enabled, and we will now check to see if
            // the Node has been home. Check the Node to see if it has already
            // been homed, 
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
                //define a timeout in case the node is unable to enable
                timeout = myMgr.TimeStampMsec() + TIME_TILL_TIMEOUT;
                // Basic mode - Poll until disabled
                while (!theNode.Motion.Homing.WasHomed()) {
                    if (myMgr.TimeStampMsec() > timeout) {
                        printf("Node did not complete homing:  \n\t"
                            "-Ensure Homing settings have been defined through"
                            " ClearView. \n\t -Check for alerts/Shutdowns \n\t"
                            " -Ensure timeout is longer than the longest" 
                            "possible homing move.\n");
                        msgUser("Press any key to continue."); 
                        return -2;
                    }
                }
                printf("Node completed homing\n");
            } 
            /* else { printf("Node[%d] has not had homing setup through ClearView.  " */
            /*         "The node will not be homed. Zeroing Position\n", iNode); */
            /*     theNode.Motion.AddToPosition( */
            /*         -(double)theNode.Motion.PosnCommanded); */
            /* } */

        }

        ///////////////////////////////////////////////////////////////////////
        //At this point we will execute 10 rev moves sequentially on each axis
        ///////////////////////////////////////////////////////////////////////
        /* INode &Node1 = myPort.Nodes(0);
        //Set the units for Acceleration to RPM/SEC
        Node1.AccUnit(INode::RPM_PER_SEC);		
        //Set the units for Velocity to RPM
        Node1.VelUnit(INode::RPM);						
        Node1.Motion.PosnMeasured.AutoRefresh(true);
        for(size_t j=0; j < 50;j++) {
            //Set Acceleration Limit (RPM/Sec)
            Node1.Motion.AccLimit = ACC_LIM_NODE_1;
            //Set Velocity Limit (RPM)
            Node1.Motion.VelLimit = VEL_SHORT_NODE_1;	
            // set this for a "HEAD" move:
            //Node1.Motion.Adv.HeadDistance = 10000;		//sets the head distance to 12,800 quad counts
            //Node1.Motion.Adv.TailDistance = 0;			//sets the tail distance to 12,800 quad counts				

            // set this for a "TAIL" move:
            //Node1.Motion.Adv.HeadDistance = 0;		//sets the head distance to 12,800 quad counts
            //Node1.Motion.Adv.TailDistance = 10000;		//sets the tail distance to 12,800 quad counts

            // set this for a "HEAD" & "TAIL" move:
            //Node1.Motion.Adv.HeadDistance = 10000;		//sets the head distance to 12,800 quad counts
            //Node1.Motion.Adv.TailDistance = 10000;		//sets the tail distance to 12,800 quad counts

            //Node1.Motion.Adv.HeadTailVelLimit = 500;	//sets the tail distance to 12,800 quad counts

            Node1.Motion.Adv.DecelLimit = 5000;
            for (size_t i = 0; i < NUM_MOVES; i++) {
                printf("Moving Nodes...Current Positions: \n");
                //Node1.Motion.Adv.MovePosnHeadTailStart(
                //    MOVE_DISTANCE_NODE_1 / NUM_MOVES);
                //Execute 10000 encoder count move
                Node1.Motion.MovePosnStart(MOVE_DISTANCE_NODE_1 / NUM_MOVES);			 
                //define a timeout in case the node is unable to enable
                double timeout = myMgr.TimeStampMsec() + 5000;			
                while (!Node1.Motion.MoveIsDone()) {
                    if (myMgr.TimeStampMsec() > timeout) {
                        printf("Error: Timed out waiting for move to complete\n");
                        msgUser("Press any key to continue."); 
                        return -2;
                    }
                    printf("Node 0: %.1f\r", (double)Node1.Motion.PosnMeasured);
                }
                printf("Node 0: %.1f\n", (double)Node1.Motion.PosnMeasured);

                printf("Move completed\n");
                myMgr.Delay(DWELL_SHORT);
            } // for each move

            myMgr.Delay(DWELL_SHORT);
            //Set Velocity Limit (RPM)
            Node1.Motion.VelLimit = VEL_LONG_NODE_1;
            printf("Returning to original position...");
            //Node1.Motion.Adv.MovePosnHeadTailStart(0, true);
            // Execute 10000 encoder count move 				
            //Node1.Motion.Adv.MovePosnAsymStart(0, true);
            // Execute 10000 encoder count move 				
            Node1.Motion.MovePosnStart(0, true); 
            double timeout = myMgr.TimeStampMsec() + 5000;
            while (!Node1.Motion.MoveIsDone()) {
                if (myMgr.TimeStampMsec() > timeout) {
                    printf("Error: Timed out waiting for move to complete\n");
                    msgUser("Press any key to continue."); 
                    return -2;
                }
                printf("Node 0: %.1f\r", (double)Node1.Motion.PosnMeasured);
            }
            printf("Node 0: %.1f\n", (double)Node1.Motion.PosnMeasured);

            printf("Move completed\n");
            myMgr.Delay(DWELL_LONG);
        }*/


        ///////////////////////////////////////////////////////////////////////
        //After moves have completed Disable node, and close ports
        ///////////////////////////////////////////////////////////////////////
        printf("Disabling nodes, and closing port\n");
        //Disable Nodes
        for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
            // Create a shortcut reference for a node
            myPort.Nodes(iNode).EnableReq(false);
        }
    } catch (mnErr& theErr) {
        printf("Failed to disable Nodes n\n");
        // This statement will print the address of the error, the error code
        // (defined by the mnErr class),as well as the corresponding error message.
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n",
                theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        msgUser("Press any key to continue.");
        return 0;  //This terminates the main program
    }

    // Close down the ports
    myMgr.PortsClose();

    msgUser("Press any key to continue.");
    return 0;			//End program
}

