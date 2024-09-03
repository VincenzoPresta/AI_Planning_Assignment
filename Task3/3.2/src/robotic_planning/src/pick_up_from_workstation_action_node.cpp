#include <memory>
#include <algorithm>
#include<vector>
#include<string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class PickUpFromWorkstation : public plansys2::ActionExecutorClient
{
public:
  PickUpFromWorkstation()
  : plansys2::ActionExecutorClient("pick_up_from_workstation", 1000ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  { std :: vector <std :: string > arguments = get_arguments () ;
    if (progress_ < 1.0) {
      progress_ += 0.5;
      send_feedback(progress_, "agent "+arguments [0]+ " is picking up "+ 
            arguments [3]+ " in "+ arguments [1]);
    } else {
      finish(true, progress_, "agent "+arguments [0]+ "picked up"+ 
            arguments [3]);

      progress_ = 0.0;
      std::cout << std::endl;
    }

    
    std::cout << arguments [0]+ " is picking up "+ 
            arguments [3]+ " in "+ arguments [1] +" . . . [ " << std::min(100.0, progress_ * 100.0) << "% ]" <<
            std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickUpFromWorkstation>();

  node->set_parameter(rclcpp::Parameter("action_name", "pick_up_from_workstation"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
