#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "deliverybots_interfaces/srv/set_integers.hpp"

using namespace std::chrono_literals;

class FactoryTester : public rclcpp::Node
{
  public:
    FactoryTester()
    : Node("factory_tester_node")
    {
      set_integer_client_ = this->create_client<deliverybots_interfaces::srv::SetIntegers>("set_integers");
      timer_ = this->create_wall_timer(1000ms,std::bind(&FactoryTester::timer_callback, this));
      sec_count = 1000;
      fire_sec = 2 + rand()%4;//180 + rand()%420;
    }
    
  protected:
    int sec_count;
    int fire_sec;

  private:
    void timer_callback()
    {
    sec_count++;
    if (sec_count>fire_sec){
        sec_count = 0;
        fire_sec = 2 + rand()%4;//180 + rand()%420;
        auto request = std::make_shared<deliverybots_interfaces::srv::SetIntegers::Request>();
        request->input_integer = 1 + rand() % 6; // random service call between 1 and 6 inclusive
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Called: %d", request->input_integer); 
        const auto future = set_integer_client_->async_send_request(request,std::bind(&FactoryTester::service_response_callback,this, std::placeholders::_1));
        }
    }

    void service_response_callback(rclcpp::Client<deliverybots_interfaces::srv::SetIntegers>::SharedFuture future) {
        this->service_response_ = future.get();  // Save service response to the pointer declared outside the scope for further process
        if (this->service_response_->success){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success"); 
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Something is up. Check middleware, then check server");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<deliverybots_interfaces::srv::SetIntegers>::SharedPtr set_integer_client_;
    deliverybots_interfaces::srv::SetIntegers::Response::SharedPtr service_response_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FactoryTester>());
  rclcpp::shutdown();
  return 0;
}