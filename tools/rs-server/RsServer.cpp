// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.
// Copyright(c) 2024 SpexAI. All Rights Reserved.
// SpexAI spex-net branch: for multiple devices

#include <iostream>

#include <cstring>
#include <liveMedia.hh>
#include <GroupsockHelper.hh>
#include <signal.h>
#include "RsUsageEnvironment.h"
#include "RsSource.hh"
#include "RsServerMediaSubsession.h"
#include "RsDevice.hh"
#include "RsRTSPServer.hh"
#include "RsServerMediaSession.h"
#include "RsCommon.h"
#include <compression/CompressionFactory.h>

#include "tclap/CmdLine.h"
#include "tclap/ValueArg.h"

using namespace TCLAP;

struct server
{
    rs2::device selected_device;
    RsRTSPServer* rtspServer;
    UsageEnvironment* env;
    std::shared_ptr<RsDevice> rsDevice;
    std::vector<rs2::video_stream_profile> supported_stream_profiles; // streams for extrinsics map creation
    std::vector<RsSensor> sensors;
    TaskScheduler* scheduler;
    unsigned int port = 8554;

    void main(int argc, char** argv)
    {
        unsigned int nth_device = 0;
        bool rgb_only = false;
        int have_streams = 0;

        START_EASYLOGGINGPP(argc, argv);

        CmdLine cmd("LRS Network Extensions Server (extended by SpexAI)", ' ', RS2_API_VERSION_STR);

        SwitchArg arg_enable_compression("c", "enable-compression", "Enable video compression");
        SwitchArg arg_rgb_only("r", "rgb-only", "Only RGB cameras");
        ValueArg<std::string> arg_address("i", "interface-address", "Address of the interface to bind on (default: 0.0.0.0)", false, "", "string");
        ValueArg<unsigned int> arg_port("p", "port", "RTSP port to listen on (default: 8554)", false, 8554, "integer");
        ValueArg<unsigned int> arg_device("d", "device", "Enable n-th device only, starting with 1", false, 0, "integer");

        cmd.add(arg_enable_compression);
        cmd.add(arg_rgb_only);
        cmd.add(arg_address);
        cmd.add(arg_port);
        cmd.add(arg_device);

        cmd.parse(argc, argv);

        CompressionFactory::getIsEnabled() = 0;
        if (arg_enable_compression.isSet())
        {
            CompressionFactory::getIsEnabled() = 1;
        }
        if (arg_rgb_only.isSet())
        {
            rgb_only = arg_rgb_only.getValue();
        }

        if (arg_address.isSet()) 
        {
            ReceivingInterfaceAddr = inet_addr(arg_address.getValue().c_str());
        }
        else
        {
            ReceivingInterfaceAddr = inet_addr("0.0.0.0");
        }

        if (arg_port.isSet())
        {
            port = arg_port.getValue();
        }
        if (arg_device.isSet())
        {
            nth_device = arg_device.getValue();
        }
        
        OutPacketBuffer::increaseMaxSizeTo(MAX_MESSAGE_SIZE);
        
        // Begin by setting up our usage environment:
        scheduler = BasicTaskScheduler::createNew();
        env = RSUsageEnvironment::createNew(*scheduler);

        rsDevice = std::make_shared<RsDevice>(env);
        rtspServer = RsRTSPServer::createNew(*env, rsDevice, port);

        if(rtspServer == NULL)
        {
            *env << "Failed to create RTSP server: " << env->getResultMsg() << "\n";
            std::cerr << "Failed to create RTSP server: " << env->getResultMsg() << "\n";
            exit(1);
        }

        sensors = rsDevice.get()->getSensors();
        unsigned i = 0;
        char old_serial[80] = { 0 };
        for(auto sensor : sensors)
        {
            char sessname[60];
            RsServerMediaSession* sms;
            const char *serial = sensor.getDevice().get_info( RS2_CAMERA_INFO_SERIAL_NUMBER );
            if (strcmp(serial, old_serial)) { // if new
                i++;
                strncpy(old_serial, serial, sizeof(old_serial) - 1);
            }
            if (nth_device > 0 && nth_device != i)
                continue;
            if((sensor.getSensorName().compare(STEREO_SENSOR_NAME) == 0) && !rgb_only)
            {
                snprintf(sessname, sizeof(sessname), "%u/depth", i);
                sensor.setSessionName(sessname);
                sms = RsServerMediaSession::createNew(*env, sensor, sessname, "", "Session streamed by \"realsense streamer\"", False);
            }
            else if(sensor.getSensorName().compare(RGB_SENSOR_NAME) == 0)
            {
                snprintf(sessname, sizeof(sessname), "%u/rgb", i);
                sensor.setSessionName(sessname);
                sms = RsServerMediaSession::createNew(*env, sensor, sessname, "", "Session streamed by \"realsense streamer\"", False);
            }
            else
            {
                continue;
            }

            for(auto stream_profile : sensor.getStreamProfiles())
            {
                rs2::video_stream_profile stream = stream_profile.second;
                if(stream.format() == RS2_FORMAT_BGR8 
                || stream.format() == RS2_FORMAT_RGB8 
                || stream.format() == RS2_FORMAT_Z16 
                || stream.format() == RS2_FORMAT_Y8 
                || stream.format() == RS2_FORMAT_YUYV 
                || stream.format() == RS2_FORMAT_UYVY)
                {
                    if(stream.fps() == 6)
                    {
                        if((stream.width() == 1280 && stream.height() == 720) || (stream.width() == 640 && stream.height() == 480)) 
                        {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));

                            supported_stream_profiles.push_back(stream);
                            continue;
                        }
                    }
                    else if(stream.fps() == 15 || stream.fps() == 30)
                    {
                        if((stream.width() == 1280 && stream.height() == 720) || (stream.width() == 640 && stream.height() == 480))
                        {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));
                            supported_stream_profiles.push_back(stream);
                            continue;
                        }
                    }
                    else if(stream.fps() == 60)
                    {
                        if((stream.width() == 640 && stream.height() == 480))
                        {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));
                            supported_stream_profiles.push_back(stream);
                            continue;
                        }
                    }
                    else if(stream.fps() == 10)
                    {
                         if(stream.format() == RS2_FORMAT_YUYV && stream.width() == 1280 && stream.height() == 720)
                         {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));
                            supported_stream_profiles.push_back(stream);
                            continue;
                         }
                    }
                }
                if (stream.format() == RS2_FORMAT_Z16 || stream.format() == RS2_FORMAT_Y8 )
                {
                    if(stream.fps() == 6 || stream.fps() == 15 || stream.fps() == 30)
                    {
                        if(stream.width() == 848 && stream.height() == 480)
                        {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));
                            supported_stream_profiles.push_back(stream);
                            continue;
                        }
                    }
                    if(stream.fps() == 30 || stream.fps() == 60)
                    {
                        if(stream.width() == 640 && stream.height() == 360)
                        {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));
                            supported_stream_profiles.push_back(stream);
                            continue;
                        }
                    }
                }
                if (stream.format() == RS2_FORMAT_YUYV)
                {
                    if(stream.fps() == 6 || stream.fps() == 15 || stream.fps() == 30 || stream.fps() == 60)
                    {
                        if(stream.width() == 424 && stream.height() == 240)
                        {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));
                            supported_stream_profiles.push_back(stream);
                            continue;
                        }
                    }
                }
            if (stream.format() == RS2_FORMAT_Z16 || stream.format() == RS2_FORMAT_Y8 || stream.format() == RS2_FORMAT_UYVY)
            {
                if(stream.fps() == 6 || stream.fps() == 15 || stream.fps() == 30 || stream.fps() == 60 ||  stream.fps() == 90)
                    {
                        if(stream.width() == 480 && stream.height() == 270)
                        {
                            sms->addSubsession(RsServerMediaSubsession::createNew(*env, stream, rsDevice));
                            supported_stream_profiles.push_back(stream);
                            continue;
                        }
                    }
            }

                *env << "Ignoring stream: format: " << stream.format() << " width: " << stream.width() << " height: " << stream.height() << " fps: " << stream.fps() << "\n";
                //std::cerr << "  -- Ignoring stream: format: " << stream.format() << " width: " << stream.width() << " height: " << stream.height() << " fps: " << stream.fps() << "\n";
            }

            calculate_extrinsics();

            rtspServer->addServerMediaSession(sms);
            char* url = rtspServer->rtspURL(sms);
            *env << "Play this stream using the URL " << url << "\n";
            std::cout << "Play this stream using the URL " << url << "\n";
            have_streams++;

            // query camera options
            rtspServer->setSupportedOptions(sessname, sensor.getSupportedOptions());

            delete[] url;
        }

        if (have_streams) {
            std::cout << "Starting rs-server...\n";
            env->taskScheduler().doEventLoop(); // does not return
        }
        else
            std::cerr << "No streams created\n";
    }

    void calculate_extrinsics()
    {
        for(auto stream_profile_from : supported_stream_profiles)
        {
            for(auto stream_profile_to : supported_stream_profiles)
            {
                int from_sensor_key = RsDevice::getPhysicalSensorUniqueKey(stream_profile_from.stream_type(), stream_profile_from.stream_index());
                int to_sensor_key = RsDevice::getPhysicalSensorUniqueKey(stream_profile_to.stream_type(), stream_profile_to.stream_index());

                rsDevice.get()->minimal_extrinsics_map[std::make_pair(from_sensor_key, to_sensor_key)] = stream_profile_from.get_extrinsics_to(stream_profile_to);
            }
        }
    }

    void sigint_handler(int sig)
    {
        Medium::close(rtspServer);
        env->reclaim();
        env = NULL;
        delete scheduler;
        scheduler = NULL;
        std::cout << "rs-server downloading\n";
    }

    // Make server a proper singleton
    // Otherwise, initialization of RsDevice is crashing
    // in static build because of order of initialization
    static server& instance()
    {
        static server inst;
        return inst;
    }
};

void sigint_handler(int sig);

int main(int argc, char** argv)
{
    server::instance().main(argc, argv);
    return 0;
}

void sigint_handler(int sig)
{
    server::instance().sigint_handler(sig);
    exit(sig);
}
