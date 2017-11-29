#ifndef __ITIA_ROS_UTILS_LOG__
# define __ITIA_ROS_UTILS_LOG__

# include <ros/console.h>
namespace itia
{ 
namespace rutils
{
 
inline std::string big_string(const std::string& in_str, int size = 60)
{
  int in_str_size = in_str.size();
  size = std::max(size, in_str_size+10);
  
  int first_half_size = (size-in_str_size-1)/2;
  int second_half_size = (size-first_half_size-in_str_size-2);
  std::string str_1; str_1.resize(size);
  std::string str_2; str_2.resize(first_half_size);
  std::string str_3; str_3.resize(second_half_size);
  
  for (int idx = 0;idx<size;idx++)
    str_1.at(idx) ='=';
    
  for (int idx = 0;idx<first_half_size;idx++)
    str_2.at(idx) ='>';
  for (int idx = 0;idx<second_half_size;idx++)
    str_3.at(idx) ='<';
  
  std::string str = str_1+ "\n" +
                    str_2 + " " + in_str + " " + str_3 +"\n" +
                    str_1;
                    
  return str;
};
}
}


# endif