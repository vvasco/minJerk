/*
* Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
* Author: Ugo Pattacini
* email:  ugo.pattacini@iit.it
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include <string>
#include <mutex>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include "Controller.h"                   // Model's header file
#include "rtwtypes.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/******************************************************/
class CtrlModule: public RFModule
{
    PolyDriver        drv;
    IControlMode     *imod;
    IEncoders        *ienc;    
    IVelocityControl *ivel;
    int joint;
    
    BufferedPort<Bottle> dataIn;
    BufferedPort<Vector> dataOut;
    RpcServer            rpc;
    mutex                mtx;

    ControllerModelClass Controller;
    ControllerModelClass::ExtU_Controller_T Controller_U;
    ControllerModelClass::ExtY_Controller_T Controller_Y;

public:
    /******************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("yarpMinJerk")).asString();
        string robot=rf.check("robot",Value("icubSim")).asString();
        string part=rf.check("part",Value("left_arm")).asString();
        joint=rf.check("joint",Value(3)).asInt();

        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/"+part);
        option.put("local","/"+name+"/"+part);
        if (!drv.open(option))
            return false;

        drv.view(imod);
        drv.view(ienc);
        drv.view(ivel);

        imod->setControlMode(joint,VOCAB_CM_VELOCITY);

        IControlLimits *ilim; drv.view(ilim);
        double min_joint,max_joint;
        ilim->getLimits(joint,&min_joint,&max_joint);

        double enc;
        while (!ienc->getEncoder(joint,&enc))
            Time::delay(0.1);

        Plant_IC=enc;
        Plant_Max=max_joint;
        Plant_Min=min_joint;
        AutoCompensator_ThresHystMax=0.5;
        AutoCompensator_ThresHystMin=0.1;

        yInfo("enc=%g in [%g, %g] deg",enc,min_joint,max_joint);       

        // Initialize model
        Controller.initialize();

        Controller_U.reference=enc;
        Controller_U.compensator_state=CompensatorState::Off;
        Controller_U.plant_output=enc;
        
        dataIn.open("/"+name+"/data:i");
        dataOut.open("/"+name+"/data:o");
        rpc.open("/"+name+"/rpc");
        attach(rpc);
        
        return true;
    }

    /******************************************************/
    double getPeriod()
    {
        return 0.01;
    }
    
    /******************************************************/
    bool updateModule()
    {
        lock_guard<mutex> lg(mtx);

        if (Bottle *in=dataIn.read(false))
            Controller_U.reference=in->get(0).asDouble();
        ienc->getEncoder(joint,&Controller_U.plant_output);

        // Step the model
        double t0=Time::now();
        // Step the model
        Controller.step();

        double t1=Time::now();

        Controller.setExternalInputs(&Controller_U);

        Controller_Y=Controller.getExternalOutputs();
        ivel->velocityMove(joint,Controller_Y.controller_output);

        Vector &out=dataOut.prepare();
        out.resize(7);
        out[0]=Controller_U.reference;
        out[1]=Controller_Y.plant_reference;
        out[2]=Controller_U.plant_output;
        out[3]=Controller_Y.controller_reference;
        out[4]=Controller_Y.controller_output;
        out[5]=Controller_Y.error_statistics;
        out[6]=Controller_Y.enable_compensation;
        dataOut.writeStrict();

        yInfo("time elapsed = %g [us]",(t1-t0)*1e6);

        return true;
    }

    /******************************************************/
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        lock_guard<mutex> lg(mtx);

        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");
        int on=Vocab::encode("on");
        int off=Vocab::encode("off");
        int automatic=Vocab::encode("auto");
        int status=Vocab::encode("status");
        int Kp=Vocab::encode("Kp");
        int Ki=Vocab::encode("Ki");

        int sw=cmd.get(0).asVocab();
        if ((sw==on) || (sw==off) || (sw==automatic))
        {
            if (sw==on)
                Controller_U.compensator_state=CompensatorState::On;
            else if (sw==off)
                Controller_U.compensator_state=CompensatorState::Off;
            else
                Controller_U.compensator_state=CompensatorState::Auto;
            reply.addVocab(ack);
            return true;
        }
        else if (sw==status)
        {
            reply.addVocab(ack);
            if (Controller_U.compensator_state==CompensatorState::On)
                reply.addVocab(on);
            else if (Controller_U.compensator_state==CompensatorState::Off)
                reply.addVocab(off);
            else
                reply.addVocab(automatic);
            return true;
        }
        else if (sw==Kp)
        {
            if (cmd.size()>1)
            {
                if (cmd.get(1).isDouble())
                {
                    Compensator_Kp=cmd.get(1).asDouble();
                    reply.addVocab(ack);
                }
                else
                    reply.addVocab(nack);
            }
            else
            {
                reply.addVocab(ack);
                reply.addDouble(Compensator_Kp);
            }
            return true;
        }
        else if (sw==Ki)
        {
            if (cmd.size()>1)
            {
                if (cmd.get(1).isDouble())
                {
                    Compensator_Ki=cmd.get(1).asDouble();
                    reply.addVocab(ack);
                }
                else
                    reply.addVocab(nack);
            }
            else
            {
                reply.addVocab(ack);
                reply.addDouble(Compensator_Ki);
            }
            return true;
        }
        else
            return RFModule::respond(cmd,reply);
    }

    /******************************************************/
    bool close()
    {
        // Terminate model
        Controller.terminate();

        imod->setControlMode(joint,VOCAB_CM_POSITION);
        
        rpc.close();
        dataOut.close();
        dataIn.close();        
        drv.close();

        return true;
    }
};


/******************************************************/
int main(int argc, char *argv[])
{   
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("Yarp network seems unavailable!");
        return -1;
    }
    
    CtrlModule mod;
    ResourceFinder rf;
    rf.configure(argc,argv);
    return mod.runModule(rf);
}

