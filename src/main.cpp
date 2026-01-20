#include <panda_cli_controller/panda_controller.hpp>

#include <ros/ros.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_cli_controller");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  PandaCliController node;

  std::string line;
  while (ros::ok())
  {
    std::cout << "\n> " << std::flush;
    if (!std::getline(std::cin, line))
      break;

    node.handleCommands(line);
  }

  ros::shutdown();
  return 0;
}
