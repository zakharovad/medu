#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "core_msgs/msg/servo_control.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::vector;
using std::array;
using std::string;

class JoysticBridge : public rclcpp::Node
{
 public:
    JoysticBridge()
    : Node("joystick_gripper")
    {
        auto joy_callback = [this](sensor_msgs::msg::Joy msg) -> void {
            std::stringstream result;
            std::copy(msg.buttons.begin(), msg.buttons.end(), std::ostream_iterator<int>(result, " "));
            //RCLCPP_INFO(this->get_logger(), "JoyMessage: '%s'", result.str().c_str());
            this->onJoystickMessage(msg);
        };
        publisher_ = this->create_publisher<core_msgs::msg::ServoControl>("/i2c_controller/servo_control", 10);
        subscription_ =  this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, joy_callback);
        this->gripperSendMessage();
    }
private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<core_msgs::msg::ServoControl>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    int gripper_progress_ = 0;
    int gripper_rotation_ = 0;

    void gripperSendMessage(){
	    auto message = core_msgs::msg::ServoControl();

	    message.gripper = this->gripper_progress_;
	    message.rotation = this->gripper_rotation_;
	    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.names[0].c_str());
	    this->publisher_->publish(message);
    };
    void onJoystickMessage(sensor_msgs::msg::Joy msg){
        int _flags = 0b00000000;
        if(msg.buttons[0] != msg.buttons[1]){
            _flags |= 1 << 0;
            this->gripper_progress_ = this->updateProgress(this->gripper_progress_,msg.buttons[0],msg.buttons[1]);
        }
        if(msg.axes[0] != 0){
            _flags |= 1 << 1;
            this->gripper_rotation_ = this->updateRotation(this->gripper_rotation_,msg.axes[0]);
        }

        if(_flags > 0){
            this->gripperSendMessage();
        }
    };
    int updateProgress(int current, int up, int down, int coeff=5){
        current += up*coeff;
        current -= down*coeff;

        if(current < -60){
            current = -60;
        }

        if(current > 60){
            current = 60;
        }
        return current;
    };
    int updateRotation(int current,  double push_stick=0){
        double coeff = (push_stick  - (-0.99999)) * (5.0 - (-5.0)) / (0.99999 - (-0.99999)) + (-5.0);
        current += std::ceil(coeff);

        if(current < -65){
            current = -65;
        }

        if(current > 65){
            current = 65;
        }
        return current;
    };

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoysticBridge>());
    rclcpp::shutdown();
    return 0;
}
