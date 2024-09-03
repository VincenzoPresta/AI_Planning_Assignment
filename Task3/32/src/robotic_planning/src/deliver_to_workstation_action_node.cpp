#include <memory>
#include <algorithm>
#include<vector>
#include<string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DeliverToWorkstation : public plansys2::ActionExecutorClient
{

public: 

	DeliverToWorkstation()
	: plansys2::ActionExecutorClient("deliver_to_workstation", 1000ms)
  	{

    progress_ = 0.0;

  	}

private:
  void do_work()
  { std :: vector <std :: string > arguments = get_arguments () ;
    if (progress_ < 1.0) {
      progress_ += 0.5;
      send_feedback(progress_, "agent "+arguments [0]+ " is delivering "+ 
            arguments [3]+ " in "+ arguments [1]);
    } else {
      finish(true, progress_, "agent "+arguments [0]+ "delivered"+ 
            arguments [3]+ " in "+ arguments [1]);

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "agent "+arguments [0]+ " is delivering "+ 
            arguments [3]+ " in "+ arguments [1]+" . . . [ " << std::min(100.0, progress_ * 100.0) << "% ] \n " <<
            std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeliverToWorkstation>();

  node->set_parameter(rclcpp::Parameter("action_name", "deliver_to_workstation"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;


}
