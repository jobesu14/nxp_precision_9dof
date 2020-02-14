#include <iostream>
#include <sstream>
#include <unistd.h>
#include <cstring>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"


/* 
APIs to be accessed by the script.
*/

static void foo_api(std::ostream &os, const std::string &arg)
{
    os << "Foo was called with arg " << arg;
}

static void bar_api(std::ostream &os, const std::string &arg)
{
    os <<"Bar was called with arg " << arg;
}

/* end API section */


/* return true if val is set, false for EOF */
static bool read_uint32(int read_fd, uint32_t &val)
{
    unsigned char msgSizeBuf[4];
    unsigned iBuf = 0;

    while (iBuf < sizeof(msgSizeBuf))
    {
        ssize_t rc = ::read(read_fd, msgSizeBuf + iBuf, sizeof(msgSizeBuf) - iBuf);

        if (rc == 0)
        {
            return false;
        }
        else if (rc < 0 )
        {
            std::cout << __func__ << "@" << __LINE__ << ":::Read ERROR" << std::endl;
            exit(1);
        }
        else
        {
            iBuf += rc;
        }
    }

    val = *(static_cast<uint32_t *>(static_cast<void *>(&msgSizeBuf[0])));
    
    return true;
}


static void send_msg(int write_fd, std::string msg)
{
    uint32_t msgSize = msg.size();
    unsigned char msgSizeBuf[4];

    ::memcpy(msgSizeBuf, &msgSize, sizeof(msgSize));

    unsigned iBuf = 0;
    while (iBuf < 4)
    {
        ssize_t rc = ::write(write_fd, msgSizeBuf + iBuf, sizeof(msgSizeBuf) - iBuf);
        if ( rc < 0 )
        {
            std::cout << "Error writing message size" << std::endl;
            ::exit(1);
        }
        else if ( rc == 0 )
        {
            std::cout << "rc == 0, what does that mean?" << std::endl;
            ::exit(1);
        }
        else
        {
            iBuf += rc;
        }
    }

    iBuf = 0;
    const char *msgBuf = msg.c_str();
    while (iBuf < msgSize)
    {
        ssize_t rc = ::write(write_fd, msgBuf + iBuf, msgSize - iBuf);
        if ( rc < 0 )
        {
            std::cout << "Error writing message" << std::endl;
            ::exit(1);
        }
        else if ( rc == 0 )
        {
            std::cout << "rc == 0, what does that mean?" << std::endl;
            ::exit(1);
        }
        else
        {
            iBuf += rc;
        }
    }
}

static std::string read_string(int read_fd, uint32_t sz)
{
    std::vector<char> msgBuf( sz + 1 );
    msgBuf[ sz ] = '\0';
    unsigned iBuf = 0;

    while (iBuf < sz)
    {
        ssize_t rc = ::read(read_fd, &(msgBuf[0]) + iBuf, sz - iBuf);

        if ( rc == 0 )
        {
            std::cout << __func__ << "@" << __LINE__ << ":::EOF read" << std::endl;
            exit(1);
        }
        else if ( rc < 0 )
        {
            std::cout << __func__ << "@" << __LINE__ << ":::Read ERROR during message" << std::endl;
            exit(1);
        }
        else
        {
            iBuf += rc;
        }
    }

    return std::string( &(msgBuf[0]) );
}


/*static void read_loop(int read_fd, int write_fd)
{

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("imu0", 10);
  ros::Rate loop_rate(200);

  while (ros::ok())
    {
        std::cout << "Waiting for message from python..." << std::endl;
        uint32_t apiNameSize;
        if (!read_uint32(read_fd, apiNameSize))
        {
            // EOF waiting for a message, script ended
            std::cout << "EOF waiting for message, script ended" << std::endl;
            return;
        }
        std::string apiName = read_string(read_fd, apiNameSize);
        uint32_t apiArgSize;
        if (!read_uint32(read_fd, apiArgSize))
        {
            std::cout << "EOF white reading apiArgSize" << std::endl;
            ::exit(1);
        }
        std::string apiArg = read_string(read_fd, apiArgSize);

        std::cout << "apiName: " << apiName << std::endl
                  << "apiArg:  " << apiArg << std::endl;


        // Response comes as [resultSize][resultString]
        if (apiName == "foo")
        {
            std::ostringstream os;
            foo_api(os, apiArg);
            send_msg(write_fd, os.str());
        }
        else if (apiName == "bar")
        {
            std::ostringstream os;
            bar_api(os, apiArg);
            send_msg(write_fd, os.str());
        }
        else
        {
            std::cout << "UNSUPPORTED API " << apiName << std::endl;
            send_msg(write_fd, "__BAD API__"); 
        }



    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    }
}*/

/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_cpp");

    int pipe_cpp_to_py[2];
    int pipe_py_to_cpp[2];

    if (::pipe(pipe_cpp_to_py) || ::pipe(pipe_py_to_cpp))
    {
        std::cout << "Couldn't open pipes" << std::endl;
        ::exit(1);
    }

    pid_t pid = fork();

    if ( pid == 0 )
    {
        ::close(pipe_py_to_cpp[0]);
        ::close(pipe_cpp_to_py[1]);
        std::ostringstream oss;

        oss << "export PY_READ_FD=" << pipe_cpp_to_py[0] << " && "
            << "export PY_WRITE_FD=" << pipe_py_to_cpp[1] << " && "
            << "export PYTHONUNBUFFERED=true && " // Force stdin, stdout and stderr to be totally unbuffered.
            << "python src/main.py";


        ::system(oss.str().c_str());
        ::close(pipe_py_to_cpp[1]);
        ::close(pipe_cpp_to_py[0]);

    }
    else if ( pid < 0 )
    {
        std::cout << "Fork failed." << std::endl;
        ::exit(1);
    }
    else
    {
        ::close(pipe_py_to_cpp[1]);
        ::close(pipe_cpp_to_py[0]);
        read_loop(pipe_py_to_cpp[0], pipe_cpp_to_py[1]);
        ::close(pipe_py_to_cpp[0]);
        ::close(pipe_cpp_to_py[1]);
    }

    return 0;
}

*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_cpp");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu0", 10);
  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    	sensor_msgs::Imu rosimu;

    	rosimu.header.stamp = ros::Time::now();
	rosimu.header.frame_id = "imu";

    	rosimu.angular_velocity.x = float(0);
    	rosimu.angular_velocity.y = float(0);
    	rosimu.angular_velocity.z = float(0);

    	rosimu.linear_acceleration.x = float(0);
    	rosimu.linear_acceleration.y = float(0);
    	rosimu.linear_acceleration.z = float(0);
        
	pub.publish(rosimu);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
