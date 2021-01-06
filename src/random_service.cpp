#include "ros/ros.h"
#include "causa_final_assignment/Random.h"
#include <math.h>



///Generates random targets among those possible
bool random_pos (causa_final_assignment::Random::Request &req,causa_final_assignment::Random::Response &res)//computes the random target
{
 float x_array[] = {-4,-4,-4, 5, 5,5};
 float y_array[] = {-3, 2, 7,-7,-3,1};	
  
 int random_i = rand()%6;
 	
 res.x = x_array[random_i];
 res.y = y_array[random_i];
 return true;
}


///It defines service /random  
int main(int argc, char **argv)
{
 ros::init(argc, argv, "random_service");
 ros::NodeHandle n;
 ros::ServiceServer service= n.advertiseService("/random", random_pos);
 ros::spin();

 return 0;
}
