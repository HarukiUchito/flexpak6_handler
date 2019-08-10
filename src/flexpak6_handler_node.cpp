
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <unistd.h>
#include <sstream>
#include <experimental/optional>

#include "serial_stream.hpp"

struct Solution
{
    double latitude;
    double longitude;
    double height;
    Solution() : latitude(0.0), longitude(0.0), height(0.0){};
    Solution(double lat, double lon, double h) : latitude(lat), longitude(lon), height(h){};
};

class Flexpak6Handler
{
public:
    Flexpak6Handler() : mLastConnected(false)
    {
        ros::NodeHandle nh("~");

        // Initialize ros objects
        mSerialSub = nh.subscribe("Serial_out", 1, &Flexpak6Handler::serial_callback, this);
        mSerialPub = nh.advertise<std_msgs::String>("Serial_in", 1);
        mTimer = nh.createTimer(ros::Duration(0.01), &Flexpak6Handler::timer_callback, this);

        // Initialize diagnostics objects
        mUpdater.setHardwareID("SerialPort");
        mUpdater.add("Connect", boost::bind(&Flexpak6Handler::diagnostic0, this, _1));

        // Get and set device path of serial port
        mDevicePath = "/dev/ttyUSB0";
        nh.getParam("device_name", mDevicePath);
        mSS.set_name(mDevicePath);
        // Open serial port
        mSS.ss_open();
        if (mSS.ss_connected())
            ROS_INFO("Serial Open %s", mDevicePath.c_str());
        else
            ROS_ERROR("Serial Fail: cound not open %s", mDevicePath.c_str());
        mLastConnected = mSS.ss_connected();

        // Receiver Initialization command
        send_command("unlogall true");
        send_command("log gpgga ontime 1");
        //send_command("log gpgll ontime 1");
    }
    ~Flexpak6Handler()
    {
        send_command("unlogall"); // Stop output from receiver
        mSS.ss_close();
    }

    void mainLoop()
    {
        ros::spin();
    }

private:
    std::string mDevicePath;
    serial_stream mSS;

    ros::Subscriber mSerialSub;
    ros::Publisher mSerialPub;
    ros::Timer mTimer;
    diagnostic_updater::Updater mUpdater;

    std::string mBuffer;

    bool mLastConnected;

    void send_command(const std::string c)
    {
        std::string command = c + '\r' + '\n';
        mSS.ss_write(command);
    }

    void serial_callback(const std_msgs::String &serial_msg)
    {
        ROS_DEBUG("Sending : %s\n", serial_msg.data.c_str());
        send_command(serial_msg.data);
    }

    void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        bool serial_c = mSS.ss_connected();
        bool serial_s = mSS.ss_status();
        if (serial_c && serial_s)
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
        else if (serial_c && !serial_s)
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "No Recieve.");
        else
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Connection.");
    }

    std::experimental::optional<Solution> parseNMEA(std::string line)
    {
        std::stringstream ss(line);
        std::string seg;
        getline(ss, seg, ',');
        if (seg[0] != '$')
            return std::experimental::nullopt;
        if (seg != "$GPGGA")
            return std::experimental::nullopt;

        Solution ret;
        getline(ss, seg, ','); // utc time
        getline(ss, seg, ','); // lat DDmm.mm
        if (seg.size()) ret.latitude = stod(seg);
        getline(ss, seg, ','); // lat dir
        getline(ss, seg, ','); // lon DDmm.mm
        if (seg.size()) ret.longitude = stod(seg);
        getline(ss, seg, ','); // lon dir
        getline(ss, seg, ','); // quality
        getline(ss, seg, ','); // sats
        getline(ss, seg, ','); // hdop
        getline(ss, seg, ','); // altitude
        if (seg.size()) ret.height = stod(seg);

        while (getline(ss, seg, ','))
        {
            //ROS_INFO("  %s", seg.c_str());
        }

        return Solution(0.0, 0.0, 0.0);
    }

    void timer_callback(const ros::TimerEvent &)
    {
        if (not mSS.ss_connected())
        {
            // Retry to connect
            mSS.ss_open();
            if (mSS.ss_connected())
                ROS_INFO("Serial Open %s", mDevicePath.c_str());
            else
                ROS_ERROR("Serial Fail: Connection is broken %s", mDevicePath.c_str());
        }
        else
        {
            std::string recv_data = mSS.ss_read();
            if (recv_data.size() > 0)
            {
                ROS_DEBUG("response (size: %d) : \n%s", int(recv_data.size()), recv_data.c_str());
                std_msgs::String serial_msg;
                serial_msg.data = recv_data;
                mSerialPub.publish(serial_msg);

                mBuffer += recv_data;
                ROS_DEBUG("Buffer :\n%s", mBuffer.c_str());

                std::stringstream ssBuf(mBuffer);
                std::string line;
                int cnt = 0;
                while (getline(ssBuf, line))
                {
                    if (line.size() == 0) continue;
                    if (line[0] == '<') continue; // Command Result
                    if (line[0] == '[') continue; // Port info
                    cnt++;
                    ROS_INFO("%d: %s", cnt, line.c_str());
                    auto ret = parseNMEA(line);
                    if (ret) {
                        Solution rv = ret.value();
                        ROS_INFO("parsed as : %f %f %f", rv.latitude, rv.longitude, rv.height);
                    }
                }
                mBuffer = "";
            }
        }
        mLastConnected = mSS.ss_connected();
        mUpdater.update();
        ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flexpak6_handler");

    Flexpak6Handler fh;
    fh.mainLoop();

    return 0;
}