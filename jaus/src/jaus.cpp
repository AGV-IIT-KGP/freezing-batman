#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/transport/jtcpclient.h>
#include <jaus/core/component.h>
#include <cxutils/keyboard.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"


// This is our JAUS ID
JAUS::UShort subsystem_id   = 1000;   // ID of our subsystem to use.
JAUS::Byte node_id          = 1;      // ID of our node to use.
JAUS::Byte component_id     = 3;      // ID of the our component.

const char judge_ip[] = "10.0.0.1";
JAUS::UShort judge_subsystem_id   = 42;    // ID of our subsystem to use.
JAUS::Byte judge_node_id          = 1;     // ID of our node to use.
JAUS::Byte judge_component_id     = 1;     // ID of the our component.

int initialize_jaus(JAUS::Component* c);
JAUS::Subsystem* discover_judge(JAUS::Component* c);

int main(int argc, char* argv[])
{
    
    JAUS::Component component;
    JAUS::Subsystem* judge_subsystem;
    std::cout << "Initializing JAUS component... ";
    if (initialize_jaus(&component) < 0)
    {
        std::cout << "Failed!" << std::endl;
        return -1;
    }
    std::cout << "OKkk" << std::endl;
    std::cout << "Searching for Judges' COP... ";
    if ((judge_subsystem = discover_judge(&component)) == NULL)
    {
        std::cout << "Failed!" << std::endl;
        return -1;
    }
    std::cout << "Success (Identification: " << judge_subsystem->mIdentification << ")" << std::endl;
    std::cout << "READY, We are now listening to ROS topics and updating current JAUS state..." << std::endl;
    JAUS::Management* management_service = (JAUS::Management*)component.GetService(JAUS::Management::Name);
    JAUS::Time::Stamp display_time = JAUS::Time::GetUtcTimeMs();

    while(management_service->GetStatus() != JAUS::Management::Status::Shutdown)
    {
        if (CxUtils::GetChar() == 'q')
            break;

        if (JAUS::Time::GetUtcTimeMs() - display_time > 2000)
        {
            management_service->PrintStatus(); // display a status message
            display_time = JAUS::Time::GetUtcTimeMs();
        }
    }

    component.Shutdown(); 
    return 0;
}

int initialize_jaus(JAUS::Component* c)
{
    c->AccessControlService()->SetTimeoutPeriod(0);
    JAUS::Discovery* discovery_service = c->DiscoveryService();
    discovery_service->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "Eklavya");
    discovery_service->SetNodeIdentification("ROS Primary");
    discovery_service->SetComponentIdentification("ROS-JAUS Interface");
    discovery_service->SetDiscoveryFrequency(1.0/5.0);
    if (c->Initialize(JAUS::Address(subsystem_id, node_id, component_id)) == false)
        return -1;

    return 0;
}

JAUS::Subsystem* discover_judge(JAUS::Component* c)
{
    

/*#ifdef REAL_JUDGES
    JAUS::JTCPClient* transport_service = NULL;
    transport_service = (JAUS::JTCPClient*) c->TransportService();
    transport_service->AddConnection(judge_ip,
      JAUS::Address(judge_subsystem_id, judge_node_id, judge_component_id));
#endif */
    JAUS::Subsystem* judge_subsystem = NULL;

    JAUS::Management* management_service = c->ManagementService();
    JAUS::Time::Stamp time_ms = JAUS::Time::GetUtcTimeMs();

    while (management_service->GetStatus() != JAUS::Management::Status::Shutdown)
    {
        if (JAUS::Time::GetUtcTimeMs() - time_ms < 1000)
            continue;
            JAUS::Discovery* discovery_service = c->DiscoveryService();
            JAUS::Subsystem::Map discovered_subsystems;
            discovery_service->GetSubsystems(discovered_subsystems);
            JAUS::Subsystem::Map::iterator subsystem;
            for (subsystem = discovered_subsystems.begin();subsystem != discovered_subsystems.end();
             subsystem++)
            {
                if (subsystem->first == judge_subsystem_id)
                {
                    judge_subsystem = subsystem->second;
                    break;
                }
               
        JAUS::Subsystem::DeleteSubsystemMap(discovered_subsystems);
    }
        time_ms = JAUS::Time::GetUtcTimeMs();

        if (judge_subsystem != NULL)
            break;
    }
    return judge_subsystem;
}
