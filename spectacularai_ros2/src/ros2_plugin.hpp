#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace spectacularai {
namespace ros2 {

class Ros2Wrapper : public rclcpp::Node {
protected:
    Ros2Wrapper(std::string name) : Node(name) {}
public:
  static std::unique_ptr<Ros2Wrapper> build();
  virtual ~Ros2Wrapper();
};

}
}
