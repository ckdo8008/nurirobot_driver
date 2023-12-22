#include "ros2-nurirobot-driver/nurirobot.hpp"

Nurirobot::Nurirobot()
    : Node("nurirobot_driver_node")
{

    if ((port_fd = open(PORT, O_RDWR | O_NDELAY)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port to Nurirobot");
        exit(-1); // TODO : put this again
    }

    struct termios2 tty;
    if (ioctl(port_fd, TCGETS2, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
        exit(-1);
    }    

    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~PARENB;
    tty.c_cflag		&=	~PARODD;
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CRTSCTS;
    tty.c_cflag     |=  CREAD | CLOCAL;

    tty.c_cflag     &=  ~CBAUD;
    tty.c_cflag     |=  CBAUDEX;
    tty.c_ispeed    =   38400;
    tty.c_ospeed    =   38400;
    tty.c_oflag     =   0;
    tty.c_oflag     &=  ~OPOST;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;    
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);
    tty.c_iflag 	&=  ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_lflag		&=  ~ICANON;
    tty.c_lflag     &=  ~(ECHO);
    tty.c_lflag     &=  ~ECHOE;     // Turn off echo erase (echo erase only relevant if canonical input is active)
    tty.c_lflag     &=  ~ECHONL;    //
    tty.c_lflag     &=  ~ISIG;      // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

    if (ioctl(port_fd, TCSETS2, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
        exit(-1);
    }

    // if ((port_fd = open(PORT, O_RDWR | O_NDELAY)) < 0)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Cannot open serial port to Nurirobot");
    //     exit(-1); // TODO : put this again
    // }

    // // 포트 설정
    // struct termios tty;
    // if (tcgetattr(port_fd, &tty) != 0) {
    //     // std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
    //     RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
    //     exit(-1);
    // }

    // cfsetospeed(&tty, B57600);
    // cfsetispeed(&tty, B57600);

    // tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8비트 문자 크기
    // tty.c_iflag &= ~IGNBRK; // 무시 브레이크 신호 무시
    // tty.c_lflag = 0;
    // tty.c_oflag = 0;

    // tty.c_cc[VMIN]  = 0; // 최소 읽기 문자
    // tty.c_cc[VTIME] = 5; // 0.5초 읽기 타임아웃  

    // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // XON/XOFF 플로우 컨트롤 끄기
    // tty.c_cflag |= (CLOCAL | CREAD); // 수신기 활성화 및 로컬 모드 활성화
    // tty.c_cflag &= ~(PARENB | PARODD); // 패리티 비트 없음
    // tty.c_cflag &= ~CSTOPB;
    // tty.c_cflag &= ~CRTSCTS;      

    // if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
    //     // std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
    //     RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
    //     exit(-1);
    // }

    RCLCPP_INFO(this->get_logger(), "Opened serial port to Nurirobot");
    pos_pub_ = this->create_publisher<nurirobot_msgs::msg::NurirobotPos>("nurirobot_driver_node/pos", 10);
    speed_pub_ = this->create_publisher<nurirobot_msgs::msg::NurirobotSpeed>("nurirobot_driver_node/speed", 10);
    hc_speed_pub_ = this->create_publisher<nurirobot_msgs::msg::NurirobotSpeed>("hc/speed", 10);

    hc_ctrl_pub_ = this->create_publisher<nurirobot_msgs::msg::HCControl>("hc/control", 10);
    hc_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("hc/joy", 10);

    left_pos_pub_ = this->create_publisher<nurirobot_msgs::msg::NurirobotPos>("left_wheel_pos", 10);
    right_pos_pub_ = this->create_publisher<nurirobot_msgs::msg::NurirobotPos>("right_wheel_pos", 10);

    remote_sub_ = this->create_subscription<std_msgs::msg::Bool>("nurirobot_remote",
                                                                 10, std::bind(&Nurirobot::setRemote_callback, this, std::placeholders::_1));
    subscriber_cmdTwist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Nurirobot::twistCallback, this, std::placeholders::_1));
    rawdata_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "mc_rawdata", 10,
            std::bind(&Nurirobot::byteMultiArrayCallback, this, std::placeholders::_1));        

    RCLCPP_INFO(this->get_logger(), "max_lin_vel_x: %f", max_lin_vel_x);
    RCLCPP_INFO(this->get_logger(), "max_ang_vel_z: %f", max_ang_vel_z);
    RCLCPP_INFO(this->get_logger(), "wheel_separation: %f", wheel_separation);
    RCLCPP_INFO(this->get_logger(), "wheel_radius: %f", wheel_radius);

    callback_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Nurirobot::timeCallback, this));
}

Nurirobot::~Nurirobot()
{ // Destructor implementation
    if (port_fd != -1)
        close(port_fd);
}

void Nurirobot::read()
{
    if (port_fd != -1)
    {
        uint8_t c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
        {
            protocol_recv(c);
        }

        if (r < 0 && errno != EAGAIN)
            RCLCPP_ERROR(this->get_logger(), "Reading from serial %s failed: %d", PORT, r);
    }
    // RCLCPP_INFO(this->get_logger(), "read");
}

void Nurirobot::cbFeedback()
{
    if (!remote)
        return;

    feedbackCall(0);
    usleep(1500);
    // std::this_thread::sleep_for(std::chrono::microseconds(1500));
    feedbackCall(1);
    usleep(1500);
    // std::this_thread::sleep_for(std::chrono::microseconds(1500));
    feedbackHCCall();
    usleep(1500);
    // RCLCPP_INFO(this->get_logger(), "cbFeedback");
}

std::string toHexString(const void *data, size_t size)
{
    const uint8_t *byteData = static_cast<const uint8_t *>(data);
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < size; ++i)
    {
        ss << std::setw(2) << static_cast<int>(byteData[i]) << " ";
    }
    return ss.str();
}

void Nurirobot::feedbackCall(uint8_t id)
{
    if (port_fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }

    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = id;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xa2;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to Nurirobot serial port");
    }
}

float Nurirobot::convertRange(float input)
{
    return 2.0 * (input - 512.0) / 1023.0;
}

void Nurirobot::commandRemoteStart()
{
    if (port_fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }

    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = (uint8_t)0xc0;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xb2;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to Nurirobot serial port");
    }

    // RCLCPP_INFO(this->get_logger(), "remote : Start");
    RCLCPP_INFO(this->get_logger(), "remote : Start %s", toHexString((const void *)&command, sizeof(command)).c_str());

}

void Nurirobot::feedbackHCCall()
{
    if (port_fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }

    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = (uint8_t)0xc0;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xb4;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to Nurirobot serial port");
    }

}

void Nurirobot::commandRemoteStop()
{
    if (port_fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }

    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = (uint8_t)0xc0;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xb3;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to Nurirobot serial port");
    }
    RCLCPP_INFO(this->get_logger(), "remote : Stop");
}

void Nurirobot::protocol_recv(uint8_t byte)
{
    start_frame = ((uint16_t)(byte) << 8) | prev_byte;
    // Read the start frame
    if (start_frame == START_FRAME)
    {
        // RCLCPP_INFO(this->get_logger(), "Start frame recognised");
        p = (uint8_t *)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
    }
    else if (msg_len >= 2 && msg_len < sizeof(FeedbackResponse))
    {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len > 4)
    {
        if ((unsigned int)(msg.datasize + 4) == msg_len)
        {
            p = (uint8_t *)&msg;
            uint8_t checksum = 0;
            size_t msg_size = (size_t)msg_len;

            for (size_t i = 2; i < msg_size; i++)
            {
                checksum += p[i];
            }
            checksum -= p[4];
            checksum = ~checksum;

            // 체크섬 확인
            if (checksum == p[4])
            {
                switch (msg.mode)
                {
                case 0x03:
                {
                    // 속도 제어 신호로 처리하면 피드백 성능이 떨어짐
                    /*
                    // 속도 제어 명령 수신
                    uint8_t *src = (uint8_t *)&msg;
                    SpeedCommand tmp;
                    uint8_t *dest = (uint8_t *)&tmp;
                    std::memcpy(dest, src, msg_len);

                    float speed = tmp.getValueSpeed() * 0.1f * (tmp.direction == 0 ? (tmp.id == 0 ? 1 : -1) : (tmp.id == 0 ? -1 : 1));
                    // RCLCPP_INFO(this->get_logger(), "id: %d, speed: %f",tmp.id, speed);

                    auto msgspeed = std::make_unique<nurirobot_msgs::msg::NurirobotSpeed>();
                    msgspeed->id = tmp.id;
                    msgspeed->speed = speed;
                    hc_speed_pub_->publish(*msgspeed);
                    */

                    // 제어 상태에 따라서 모드 변경이 필요
                    if (remote) {
                        commandRemoteStart();
                    }
                    break;
                }
                case 0xb1:
                {
                    // 휠체어 상태 피드백
                    uint8_t *src = (uint8_t *)&msg;
                    WheelchairResponse tmp;
                    uint8_t *dest = (uint8_t *)&tmp;
                    std::memcpy(dest, src, msg_len);

                    auto msg = std::make_unique<nurirobot_msgs::msg::HCControl>();
                    msg->xaxis = tmp.x;
                    msg->yaxis = tmp.y;
                    msg->adc = tmp.volt;
                    msg->clickbutton = tmp.btn == 4 ? false : true;
                    msg->speed = tmp.speed;
                    hc_ctrl_pub_->publish(*msg);              

                    auto joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
                    float x = convertRange(tmp.x);
                    float y = convertRange(tmp.y);

                    if (abs(x) < (DEADZONE / 1023.0)) {
                        x = 0;
                    }

                    if (abs(y) < (DEADZONE / 1023.0)) {
                        y = 0;
                    }                    

                    joy_msg->axes.push_back(x);
                    joy_msg->axes.push_back(y);
                    joy_msg->buttons.push_back(tmp.btn == 4 ? false : true);

                    joy_msg->header.stamp = this->now();

                    hc_joy_pub_->publish(*joy_msg);

                    RCLCPP_DEBUG(this->get_logger(), "y : %d, x:%d, adc: %d, btn: %d", tmp.y, tmp.x, tmp.volt, tmp.btn );
                    break;
                }
                case 0xd2:
                {
                    // 속도 피드백
                    // 현재 바퀴의 위치를 발행
                    uint8_t *src1 = (uint8_t *)&msg;
                    SpeedFeedbackResponse tmp1;
                    uint8_t *dest1 = (uint8_t *)&tmp1;
                    std::memcpy(dest1, src1, msg_len);

                    auto msgpos = std::make_unique<nurirobot_msgs::msg::NurirobotPos>();
                    msgpos->id = tmp1.id;
                    msgpos->pos = tmp1.getValuePos(); 
                    pos_pub_->publish(*msgpos);

                    auto msglrpos = std::make_unique<nurirobot_msgs::msg::NurirobotPos>();
                    msglrpos->header.stamp = this->get_clock()->now();
                    msglrpos->header.frame_id = "pos";
                    msglrpos->id = tmp1.id;
                    msglrpos->pos = tmp1.getValuePos(); 

                    if (tmp1.id == 0)
                        left_pos_pub_->publish(*msglrpos);
                    else 
                        right_pos_pub_->publish(*msglrpos);

                    auto msgspeed = std::make_unique<nurirobot_msgs::msg::NurirobotSpeed>();
                    msgspeed->id = tmp1.id;
                    msgspeed->speed = tmp1.getValueSpeed() * 0.1f * (tmp1.direction == 0 ? (tmp1.id == 0 ? 1 : -1) : (tmp1.id == 0 ? -1 : 1));
                    speed_pub_->publish(*msgspeed);



                    // if (tmp1.id == 1) {
                    //     // RCLCPP_INFO(this->get_logger(), "recv : %s",  toHexString(&msg, msg_len).c_str());
                    //     RCLCPP_INFO(this->get_logger(), "id: %d, dir: %d, pos : %d, speed:%d",tmp1.id, tmp1.direction, tmp1.getValuePos(), tmp1.getValueSpeed() );
                    //     // RCLCPP_INFO(this->get_logger(), "recv : %s",  toHexString(&tmp1, msg_len).c_str());
                    // }
                    break;
                }
                default:
                    break;
                }

                // RCLCPP_INFO(this->get_logger(), "recv : %s",  toHexString(&msg, msg_len).c_str());
            }
            else 
            {
                RCLCPP_INFO(this->get_logger(), "protocol_recv 1 : %s %d %d",  toHexString(&msg, msg_len).c_str(), checksum, p[4]);
            }
            msg_len = 0;
        }
    }

    if (msg_len == sizeof(FeedbackResponse))
    {
        RCLCPP_INFO(this->get_logger(), "recv feed: %s", toHexString(&msg, sizeof(msg)).c_str());
        msg_len = 0;
    }

    prev_byte = byte;
}

void Nurirobot::setRemote_callback(std_msgs::msg::Bool::UniquePtr msg)
{
    remote = msg->data;

    if (remote)
    {
        commandRemoteStart();
    }
    else
    {
        commandRemoteStop();
    }
    // RCLCPP_INFO(this->get_logger(), "remote : %s", remote ? "True": "False");
}

void Nurirobot::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (port_fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }

    if (!remote)
        return;

    float lin_vel_x = msg->linear.x;
    float ang_vel_z = msg->angular.z;

    // RCLCPP_INFO(this->get_logger(),
    //             "Received Twist: linear.x: '%f', angular.z: '%f'",
    //             lin_vel_x, ang_vel_z);

    lin_vel_x = std::max(-max_lin_vel_x, std::min(max_lin_vel_x, lin_vel_x));
    ang_vel_z = std::max(-max_ang_vel_z, std::min(max_ang_vel_z, ang_vel_z));

    write_base_velocity(lin_vel_x, ang_vel_z);

    // RCLCPP_INFO(this->get_logger(), "twistCallback");
}

void Nurirobot::write_base_velocity(float x, float z)
{

    std::vector<uint8_t> mutable_bytearray_left = {0xff, 0xfe, 0x00, 0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01};
    std::vector<uint8_t> mutable_bytearray_right = {0xff, 0xfe, 0x01, 0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01};

    float v = x;
    float w = z;

    double rpm_left = (v - w * wheel_separation / 2.0) * 60.0 / (2.0 * M_PI * wheel_radius);
    double rpm_right = (v + w * wheel_separation / 2.0) * 60.0 / (2.0 * M_PI * wheel_radius);

    int absleftrpm = std::floor(std::abs(rpm_left) * 10);
    int absrightrpm = std::floor(std::abs(rpm_right) * 10);

    uint8_t leftdir = (rpm_left > 0) ? 0x00 : 0x01;
    uint8_t rightdir = (rpm_right > 0) ? 0x01 : 0x00;

    mutable_bytearray_left[6] = leftdir;
    mutable_bytearray_right[6] = rightdir;

    mutable_bytearray_left[7] = (absleftrpm >> 8) & 0xFF;
    mutable_bytearray_left[8] = absleftrpm & 0xFF;
    mutable_bytearray_right[7] = (absrightrpm >> 8) & 0xFF;
    mutable_bytearray_right[8] = absrightrpm & 0xFF;

    int sum_val_l = 0;
    for (size_t i = 2; i < mutable_bytearray_left.size(); ++i)
    {
        sum_val_l += mutable_bytearray_left[i];
    }
    sum_val_l -= mutable_bytearray_left[4];

    int sum_val_r = 0;
    for (size_t i = 2; i < mutable_bytearray_right.size(); ++i)
    {
        sum_val_r += mutable_bytearray_right[i];
    }
    sum_val_r -= mutable_bytearray_right[4];

    uint8_t leftchecksum = ~sum_val_l & 0xFF;
    uint8_t rightchecksum = ~sum_val_r & 0xFF;

    mutable_bytearray_left[4] = leftchecksum;
    mutable_bytearray_right[4] = rightchecksum;

    int rc = ::write(port_fd, mutable_bytearray_left.data(), mutable_bytearray_left.size());
    if (rc < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to Nurirobot serial port");
        return;
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // usleep(1500);
    // std::this_thread::sleep_for(std::chrono::microseconds(1500));
    rc = ::write(port_fd, mutable_bytearray_right.data(), mutable_bytearray_right.size());
    if (rc < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to Nurirobot serial port");
        return;
    }
    // usleep(1500);
    // std::this_thread::sleep_for(std::chrono::microseconds(1500));
}

void Nurirobot::byteMultiArrayCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    if (port_fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }

    int rc = ::write(port_fd, msg->data.data(), msg->data.size());
    if (rc < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to Nurirobot serial port");
        return;
    }
    std::ostringstream hex_stream;

    for (auto byte : msg->data) {
        hex_stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }

    std::string hex_string = hex_stream.str();
    // RCLCPP_INFO(this->get_logger(), "size: %d, mc_rawdata : %s", msg->data.size(), hex_string);
}

void Nurirobot::timeCallback()
{
    cbFeedback();
    // std::this_thread::sleep_for(std::chrono::microseconds(2000));
    // usleep(2000);
    read();
}