#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
int
main(void)
{
  sensor_msgs::LaserScan scan;
  for(int i=0; i<100; i++)
    scan.ranges.push_back(42.0*i);

  for(std::vector<float>::const_iterator it = scan.ranges.begin();
      it != scan.ranges.end();
      ++it)
    printf("%f\n", *it);
}
