#include <memory>
#include <algorithm>
#include<vector>
#include<string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class Move : public plansys2::ActionExecutorClient
{
public:
  Move()
  : plansys2::ActionExecutorClient("move", 2500ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  { std :: vector <std :: string > arguments = get_arguments () ;
    if (progress_ < 1.0) {
      progress_ += 0.5;
      send_feedback(progress_, "agent "+arguments [0]+ " moving from "+ 
            arguments [1]+ " to "+ arguments [2]);
    } else {
      finish(true, 1.0, "agent "+arguments [0]+ " moved from"+ 
            arguments [1]+ " to "+ arguments [2]);

      progress_ = 0.0;
      std::cout << std::endl;
    }

    
    std::cout << arguments [0]+ " is moving from "+ 
            arguments [1]+ " to "+ arguments [2]+" . . . [ " << std::min(100.0, progress_ * 100.0) << "% ]" <<
            std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Move>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
