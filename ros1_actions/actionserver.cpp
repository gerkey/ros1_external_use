#include <myproject/DoDishesAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<myproject::DoDishesAction> Server;

void execute(const myproject::DoDishesGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
