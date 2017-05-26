/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <PholusTutorial_rt_plugin.h>

/* Specify that the class XBotPlugin::PholusTutorial is a XBot RT plugin with name "PholusTutorial" */
REGISTER_XBOT_PLUGIN(PholusTutorial, XBotPlugin::PholusTutorial)

namespace XBotPlugin {

bool PholusTutorial::init_control_plugin(std::string path_to_config_file,
                                                    XBot::SharedMemory::Ptr shared_memory,
                                                    XBot::RobotInterface::Ptr robot)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = robot;
    
    _robot->getRobotState("home", _q_final);

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */
    
    _logger = XBot::MatLogger::getLogger("/tmp/PholusTutorial_log");
    
    _enable_gcomp = true;

    return true;


}

void PholusTutorial::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /PholusTutorial_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);
    
    _robot->getStiffness(_k0);
    _robot->getDamping(_d0);
    
    _robot->setStiffness(_k0 * 0.01);
    _robot->setDamping(_d0 * 0.1);
    _robot->move();
    
    /* Save the robot starting config to a class member */
    _start_time = time;
}

void PholusTutorial::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /PholusTutorial_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void PholusTutorial::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
    double HOMING_TIME = 5.0;
    
    double tau = (time - _start_time)/HOMING_TIME;
    tau = std::min(tau, 1.0);
    
    _qref = (1 - tau)*_q0 + tau*_q_final;
    
    if(command.read(current_command)){
        if(current_command.str() == "ON"){
            _enable_gcomp = true;
        }
        
        if(current_command.str() == "OFF"){
            _enable_gcomp = false;
        }
    }
    
    if(_enable_gcomp){
        _robot->model().computeGravityCompensation(_gcomp);
    }
    else{
        _gcomp.setZero(_robot->getJointNum());
    }
    
    _robot->setPositionReference(_qref);
    _robot->setEffortReference(_gcomp);
    _robot->move();
    
    

}

bool PholusTutorial::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}