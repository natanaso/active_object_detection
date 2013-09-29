#ifndef _HYPOTHESIS_HPP_
#define _HYPOTHESIS_HPP_
#include <boost/lexical_cast.hpp>
class Hypothesis
{
public:
	bool positive;
	std::string name;
	double roll;
	double pitch;
	double yaw;
	
	Hypothesis( bool positive, std::string name, 
			    double r, double p, double y )
		: positive(positive), name(name), roll(r), pitch(p), yaw(y)
	{}
	
	std::string to_str() const
   {
   	if( positive )
        return name + " " + boost::lexical_cast<std::string>(roll) 
        			+ " " + boost::lexical_cast<std::string>(pitch)
        			+ " " + boost::lexical_cast<std::string>(yaw);
      else
      	return "NULL: " + name;
   }
    
private:
	friend std::ostream & operator<<(std::ostream &os, const Hypothesis& h)
	{
		return os << h.to_str();
	}	

};

#endif
