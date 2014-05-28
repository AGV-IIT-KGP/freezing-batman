
#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/transport/jtcpclient.h>
#include <jaus/core/component.h>
#include <cxutils/keyboard.h>
#include <iostream>
#include <cstring>
#include <string.h>

#include "CommandConsole.h"

#define JAUS_ID 42
#define ENTRY_JAUS_ID 1000
#define COMPONENT_ID 3 // This is our ID

/*
 * Prototypes
 */
int InitializeJAUSComponent(JAUS::Component* c);
JAUS::Subsystem* DiscoverEntrySubsystem(JAUS::Component* c);

void CmdHelp(FILE *Console, char *Args[]);
void CmdGotoStandby(FILE *Console, char *Args[]);
void CmdGotoReady(FILE *Console, char *Args[]);
void CmdShutdown(FILE *Console, char *Args[]);
void CmdQueryStatus(FILE *Console, char *Args[]);
void CmdSetLocalPose(FILE *Console, char *Args[]);
void CmdToggleVelocityMessages(FILE *Console, char *Args[]);
void CmdToggleLocalPoseMessages(FILE *Console, char *Args[]);
void CmdExit(FILE *Console, char *Args[]);
void QueryVelocity(void);
void QueryLocalPose(void);

/*
 * Important JAUS addresses
 */
JAUS::Address componentID(JAUS_ID, 1, 1);
JAUS::Address entryComponentID(ENTRY_JAUS_ID, 1,3);

/*
 * Globals
 */
JAUS::Component component;
bool queryVelocity = false;
bool queryPose = false;

/*
 * Perform all initialize of our JAUS component.
 */
int InitializeJAUSComponent(JAUS::Component* c)
{
    JAUS::Discovery* discoveryService = c->DiscoveryService();

    // set JAUS names
    discoveryService->SetSubsystemIdentification(JAUS::Subsystem::OCU,"COP");
    discoveryService->SetNodeIdentification("Judges' COP");
    discoveryService->SetComponentIdentification("Baseline");

    // discovery frequency in Hz -- rules say every 5 seconds
    discoveryService->SetDiscoveryFrequency(1.0/5.0);

    // initialize everything
    if (c->Initialize(componentID) == false)
        return -1;

    return 0;
}

/*
 * Discover the JAUS-ROS interface node.  Will not return until it is discovered or failure.
 */
JAUS::Subsystem* DiscoverEntrySubsystem(JAUS::Component* c)
{
    JAUS::Subsystem* entrySubsystem = NULL;

    // get a pointer to our Management Service
    JAUS::Management* managementService = c->ManagementService();
    JAUS::Time::Stamp timeMs = JAUS::Time::GetUtcTimeMs();

    while (managementService->GetStatus() != JAUS::Management::Status::Shutdown)
    {
        CxUtils::SleepMs(10); // save some CPU time

        if (JAUS::Time::GetUtcTimeMs() - timeMs < 1000)
            continue;

        // now look at discovered subsystems (the judges' COP)
        JAUS::Discovery* discoveryService = c->DiscoveryService();

        JAUS::Subsystem::Map discoveredSubsystems;
        discoveryService->GetSubsystems(discoveredSubsystems);

        JAUS::Subsystem::Map::iterator subsystem;
        for (subsystem = discoveredSubsystems.begin();
             subsystem != discoveredSubsystems.end();
             subsystem++)
        {
            if (subsystem->first == ENTRY_JAUS_ID)
            {   
                std::cout<<subsystem->first<<" "<<(subsystem->second)->mIdentification<<" "<<ENTRY_JAUS_ID<<std::endl;
              // if(std::strcmp(subsystem->second->mIdentification.c_str(),"Eklavya"))
                break;
            }
        }
        JAUS::Subsystem::DeleteSubsystemMap(discoveredSubsystems);
        timeMs = JAUS::Time::GetUtcTimeMs();
        entrySubsystem= subsystem->second;
        if (entrySubsystem != NULL)
            break;
    }

    return entrySubsystem;
}

/*
 * Main program entry point.
 */
int main(int argc, char* argv[])
{
    JAUS::Subsystem* entrySubsystem;

    std::cout << "Initializing Simulated Judges' COP JAUS component... ";

    if (InitializeJAUSComponent(&component) < 0)
    {
        std::cout << "Failed!" << std::endl;
        return -1;
    }

    std::cout << "OK" << std::endl;

    std::cout << "Searching for Entry JAUS component... ";

    if ((entrySubsystem = DiscoverEntrySubsystem(&component)) == NULL)
    {
        std::cout << "Failed!" << std::endl;
        return -1;
    }

    std::cout << "Success (Identification: " << entrySubsystem->mIdentification << ")" << std::endl;

    std::cout << "Type \"?\" for a list of commands." << std::endl;

    // create a command console
    CommandConsole CmdConsole("> ", NULL);

    CmdConsole.AddCommand("?", CmdHelp);
    CmdConsole.AddCommand("standby", CmdGotoStandby);
    CmdConsole.AddCommand("resume", CmdGotoReady);
    CmdConsole.AddCommand("shutdown", CmdShutdown);
    CmdConsole.AddCommand("status", CmdQueryStatus);
    CmdConsole.AddCommand("setpose", CmdSetLocalPose);
    CmdConsole.AddCommand("tv", CmdToggleVelocityMessages);
    CmdConsole.AddCommand("tp", CmdToggleLocalPoseMessages);
    CmdConsole.AddCommand("exit", CmdExit);

    CmdConsole.StartThread(STDIN_FILENO);

    // just loop forever printing updates
    JAUS::Management* managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);

    JAUS::Time::Stamp displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();

    while (true)
    {
        if (managementService->GetStatus() == JAUS::Management::Status::Shutdown)
            break;

        // request velocity update every ~2 seconds
        if (JAUS::Time::GetUtcTimeMs() - displayStatusTimeMs > 250)
        {
            if (queryVelocity)
                QueryVelocity();

            if (queryPose)
                QueryLocalPose();

            // display our management node status message
//            managementService->PrintStatus();

            displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
        }

        CxUtils::SleepMs(10);
    }

    component.Shutdown(); // this will take care of cleanup
    return 0;
}

/*
 * Keyboard command routines (registered with CommandConsole)
 */
void CmdHelp(FILE *Console, char *Args[])
{
    std::cout << "The following commands are available:" << std::endl;

    std::cout << "\t?           - This help" << std::endl;
    std::cout << "\tstandby     - Put robot into standby state" << std::endl;
    std::cout << "\tresume      - Put robot into ready state" << std::endl;
    std::cout << "\tshutdown    - Put robot into shutdown state" << std::endl;
    std::cout << "\tstatus      - Query the robot's current state" << std::endl;
    std::cout << "\tsetpose x y - Set the local pose of the robot" << std::endl;

    std::cout << "\ttv          - Toggle velocity queries" << std::endl;
    std::cout << "\ttp          - Toggle local pose queries" << std::endl;

    std::cout << "\texit        - Quit the program" << std::endl;
    std::cout << std::endl;
}

void CmdGotoStandby(FILE *Console, char *Args[])
{
    JAUS::Standby standbyMsg;
    JAUS::ReportStatus response;

    standbyMsg.SetDestinationID(entryComponentID);
    standbyMsg.SetSourceID(componentID);

    std::cout << "Sending Standby message to " << entryComponentID.ToString() << std::endl;
    component.Send(&standbyMsg, &response, 100);
}

void CmdGotoReady(FILE *Console, char *Args[])
{
    JAUS::Resume resumeMsg;
    JAUS::ReportStatus response;

    resumeMsg.SetDestinationID(entryComponentID);
    resumeMsg.SetSourceID(componentID);

    std::cout << "Sending Resume message to " << entryComponentID.ToString() << std::endl;
    component.Send(&resumeMsg, &response, 100);
}

void CmdShutdown(FILE *Console, char *Args[])
{
    JAUS::Shutdown shutdownMsg;
    JAUS::ReportStatus response;

    shutdownMsg.SetDestinationID(entryComponentID);
    shutdownMsg.SetSourceID(componentID);

    std::cout << "Sending Shutdown message to " << entryComponentID.ToString() << std::endl;
    component.Send(&shutdownMsg, &response, 100);
}

void CmdQueryStatus(FILE *Console, char *Args[])
{
    JAUS::QueryStatus queryMsg;
    JAUS::ReportStatus response;

    queryMsg.SetDestinationID(entryComponentID);
    queryMsg.SetSourceID(componentID);

    std::cout << "Sending Query message to " << entryComponentID.ToString() << "..." << std::endl;

    if (component.Send(&queryMsg, &response, 1000))
    {
        std::cout << "\tReceived Response Status: ";

        switch (response.GetStatus())
        {
            case 0:  std::cout << "Init";      break;
            case 1:  std::cout << "Ready";     break;
            case 2:  std::cout << "Standby";   break;
            case 3:  std::cout << "Shutdown";  break;
            case 4:  std::cout << "Failure";   break;
            case 5:  std::cout << "Emergency"; break;
            default: std::cout << "<Invalid>"; break;
        }

        std::cout << std::endl;
    }
    else
        std::cout << "\tNo Response!" << std::endl;
}

void CmdSetLocalPose(FILE *Console, char *Args[])
{
    JAUS::SetLocalPose setMsg;
    JAUS::ReportLocalPose response;

    setMsg.SetDestinationID(entryComponentID);
    setMsg.SetSourceID(componentID);

    // just try setting some silly pose for testing purposes
    setMsg.SetX(1.0);
    setMsg.SetY(10.0);
    setMsg.SetYaw(2.0);

    std::cout << "Sending Set Local Pose message to " << entryComponentID.ToString() << "...";

    component.Send(&setMsg, &response, 1000);
    std::cout << "OK" << std::endl;
}

void CmdToggleVelocityMessages(FILE *Console, char *Args[])
{
    queryVelocity = (queryVelocity ? false : true);
}

void CmdToggleLocalPoseMessages(FILE *Console, char *Args[])
{
    queryPose = (queryPose ? false : true);
}

void CmdExit(FILE *Console, char *Args[])
{
    std::cout << "Goodbye." << std::endl;
    exit(0);
}

/*
 * Called periodically to get the velocity from the robot.
 */
void QueryVelocity(void)
{
    static JAUS::Time lastTimeStamp = JAUS::Time::GetUtcTime();

    JAUS::QueryVelocityState queryMsg;
    JAUS::ReportVelocityState response;

    queryMsg.SetDestinationID(entryComponentID);
    queryMsg.SetSourceID(componentID);

    // setup to recieve the data the judges care about
    queryMsg.SetPresenceVector(
      JAUS::QueryVelocityState::PresenceVector::VelocityX
      | JAUS::QueryVelocityState::PresenceVector::YawRate
      | JAUS::QueryVelocityState::PresenceVector::TimeStamp);

    if (component.Send(&queryMsg, &response, 1000))
    {
        if (response.GetTimeStamp() != lastTimeStamp)
        {
//            response.Print();
            double vel = response.GetVelocityX();
            double yaw = response.GetYawRate();
            lastTimeStamp = response.GetTimeStamp();

            printf("[%u] (Velocity) X: %f, Yaw Rate: %f\n", lastTimeStamp.ToUInt(), vel, yaw);
        }
    }
}

/*

 * Called periodically to get the local pose of the robot.
 */
void QueryLocalPose(void)
{
    static JAUS::Time lastTimeStamp = JAUS::Time::GetUtcTime();

    JAUS::QueryLocalPose queryMsg;
    JAUS::ReportLocalPose response;

    queryMsg.SetDestinationID(entryComponentID);
    queryMsg.SetSourceID(componentID);

    // setup to recieve the data the judges care about
    queryMsg.SetPresenceVector(
      JAUS::QueryLocalPose::PresenceVector::X
      | JAUS::QueryLocalPose::PresenceVector::Y
      | JAUS::QueryLocalPose::PresenceVector::TimeStamp);

    if (component.Send(&queryMsg, &response, 1000))
    {
        if (response.GetTimeStamp() != lastTimeStamp)
        {
//            response.Print();

            double x = response.GetX();
            double y = response.GetY();
            lastTimeStamp = response.GetTimeStamp();

            printf("[%u] (Pose) X: %f, Y: %f\n", lastTimeStamp.ToUInt(), x, y);
        }
    }
}

/*
 * End of judge.cpp
 */

