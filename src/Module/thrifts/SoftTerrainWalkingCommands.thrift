/**
 * @file SoftTerrainWalkingCommands.thrift
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @License Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

service SoftTerrainWalkingCommands
{
    /**
     * Call this method to prepare the robot.
     * iCub will be moved in the home position and the positionDirect
     * interface will be set.
     * @return true/false in case of success/failure;
     */
    bool prepareRobot();

    /**
     * Run the entire walking controller.
     * @return true/false in case of success/failure;
     */
    bool startWalking();

}
