/******************************************************************************
File name: GazeboToArducopterSerial.cpp
Description: ROS serial interface for arducopter stepper board
Author: Antun Ivanovic
******************************************************************************/

#include <GazeboToArducopterSerial.h>

GazeboToArducopterSerial::GazeboToArducopterSerial()
{
    // Initialize private node handle for params.
    int motor_address;
    nhParams = ros::NodeHandle("~");
    nhParams.param("port", port, string("/dev/ttyUSB0"));
    nhParams.param("baudrate", baudrate, int(115200));

    // Set up node handle for topics
    all_mass_sub = nhTopics.subscribe("movable_mass_all/command", 1,
        &GazeboToArducopterSerial::allMassCallback, this);

    f = boost::bind(&GazeboToArducopterSerial::reconfigureCallback, this, _1, _2);
    server.setCallback(f);
}

GazeboToArducopterSerial::~GazeboToArducopterSerial()
{

}

void GazeboToArducopterSerial::run()
{
    cout << "Opening serial port" << endl;
    SetSerialAttributes(port, baudrate);

    cout << "Port opened, starting communication." << endl;
    ros::spin();
}

int GazeboToArducopterSerial::SetSerialAttributes(string port, int baudrate)
{
    ROS_INFO("Setting up serial port parameters.");

    // First open port
    const char *charPort = port.c_str();
    USB = open(charPort, O_RDWR | O_NOCTTY);

    memset(&tty, 0, sizeof tty);
    // Error Handling
    if (tcgetattr ( USB, &tty ) != 0) 
    {
        std::cout << "Error " << errno << " from tcgetattr: " << 
            strerror(errno) << std::endl;
        return 0;
    }

    // Save old tty parameter
    tty_old = tty;

    // Set Baud Rate. Assuming one of these will be used. Add more if needed.
    switch (baudrate)
    {
        case 9600:
            cfsetospeed (&tty, (speed_t)B9600);
            cfsetispeed (&tty, (speed_t)B9600);
            ROS_INFO("Setting baudrate to 9600.");
            break;
        case 19200:
            cfsetospeed (&tty, (speed_t)B19200);
            cfsetispeed (&tty, (speed_t)B19200);
            ROS_INFO("Setting baudrate to 19200.");
            break;
        case 38400:
            cfsetospeed (&tty, (speed_t)B38400);
            cfsetispeed (&tty, (speed_t)B38400);
            ROS_INFO("Setting baudrate to 38400.");
            break;
        case 57600:
            cfsetospeed (&tty, (speed_t)B57600);
            cfsetispeed (&tty, (speed_t)B57600);
            ROS_INFO("Setting baudrate to 57600.");
            break;
        case 115200:
            cfsetospeed (&tty, (speed_t)B115200);
            cfsetispeed (&tty, (speed_t)B115200);
            ROS_INFO("Setting baudrate to 115200.");
            break;
        case 230400:
            cfsetospeed (&tty, (speed_t)B230400);
            cfsetispeed (&tty, (speed_t)B230400);
            ROS_INFO("Setting baudrate to 230400.");
            break;
        case 460800:
            cfsetospeed (&tty, (speed_t)B460800);
            cfsetispeed (&tty, (speed_t)B460800);
            ROS_INFO("Setting baudrate to 460800.");
            break;
        case 921600:
            cfsetospeed (&tty, (speed_t)B921600);
            cfsetispeed (&tty, (speed_t)B921600);
            ROS_INFO("Setting baudrate to 921600.");
            break;
    }

    // Setting other Port Stuff
    tty.c_cflag     &=  ~PARENB; // No parity bit
    tty.c_cflag     &=  ~CSTOPB; // One stop bit
    tty.c_cflag     &=  ~CSIZE; 
    tty.c_cflag     |=  CS8;     // 8-bit characters are sent

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  1;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    // Make raw
    cfmakeraw(&tty);

    // Flush Port, then applies attributes
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) 
    {
       std::cout << "Error " << errno << " from tcsetattr" << std::endl;
       return 0;
    }

    ROS_INFO("Serial port %s successfully open!", charPort);

    return 1;
}

int GazeboToArducopterSerial::SerialWrite(int m[4], unsigned char terminator)
{   
    //if (terminator == 67) cout << "Writing motor references to serial port." << endl;
    //else if (terminator == 83) cout << "Writing parameters to serial port." << endl;
    unsigned char dataByte;
    for (int i=0; i < 4; i++)
    {
        dataByte = m[i];
        write(USB, &dataByte, 1);
        dataByte = m[i] >> 8;
        write(USB, &dataByte, 1);
        dataByte = m[i] >> 16;
        write(USB, &dataByte, 1);
        dataByte = m[i] >> 24;
        write(USB, &dataByte, 1);
    }
    dataByte = terminator;
    write(USB, &dataByte, 1);
    dataByte = 0;
    write(USB, &dataByte, 1);
    write(USB, &dataByte, 1);
    write(USB, &dataByte, 1);

    return 1;
}



int GazeboToArducopterSerial::SerialRead()
{
    int n = 0, spot = 0;
    char buf = '\0';

    /* Whole response*/
    char response[300];
    memset(response, '\0', sizeof response);

    fd_set set;
    struct timeval timeout;
    int rv;
    bool messageCheckFlag = false;
    do {
        FD_ZERO(&set); /* clear the set */
        FD_SET(USB, &set); /* add our file descriptor to the set */
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;

        rv = select(USB + 1, &set, NULL, NULL, &timeout);
        if(rv == -1)
            perror("select"); /* an error accured */
        else if(rv == 0)
        {
            printf("timeout"); /* a timeout occured */
            return -1;
        }
        else
            n = read( USB, &buf, 1 ); /* there was data to read */

        sprintf( &response[spot], "%c", buf );
        //printf("%x\n", response[spot]);
        //mis231Motor.receive_array[spot] = response[spot];
        spot += n;
        //mis231Motor.received_length = spot;

        // Checking message
        //printf("%d %d \n", MIS231_CheckWriteMessageReceived(&mis231Motor), 
        //    MIS231_CheckReadMessageReceived(&mis231Motor));
        //if(MIS231_CheckWriteMessageReceived(&mis231Motor) == 1 || 
        //    MIS231_CheckReadMessageReceived(&mis231Motor) == 1)
        //{
        //    messageCheckFlag = true;
        //}

    } while((messageCheckFlag == false) && n > 0);

    return 1;
}

void GazeboToArducopterSerial::allMassCallback(const std_msgs::Float64MultiArray &msg)
{   
    float scaler = 4500.0;
    int m[4] = {0,0,0,0};
    if (msg.data.size() < 4)
    {
        cout << "Not enough data. Length: " << msg.data.size() << endl;
    }
    else
    {
        for(int i = 0; i < msg.data.size(); i++)
        {
            m[i] = int(scaler*msg.data[i]);
            if (msg.data[i] > 0.08) m[i] = int(scaler*0.08);
            else if (msg.data[i] < -0.08) m[i] = int(-scaler*0.08);
        }
    }

    // Create big string
    m[1]*=0.0;
    m[3]*=0.0;
    SerialWrite(m, 67);
}

void GazeboToArducopterSerial::reconfigureCallback(mmuav_arducopter_bridge::StepperParametersConfig &config, uint32_t level) {
  
  int m[4] = {0,0,0,0};
  ROS_INFO("Reconfigure Request: %d %d %d %d", 
            config.gain, config.ang_speed_pps, 
            config.ang_acc_pos_ppss, config.ang_acc_neg_ppss);
  m[0] = config.gain;
  m[1] = config.ang_speed_pps;
  m[2] = config.ang_acc_pos_ppss;
  m[3] = config.ang_acc_neg_ppss;

  SerialWrite(m, 83); 

}
