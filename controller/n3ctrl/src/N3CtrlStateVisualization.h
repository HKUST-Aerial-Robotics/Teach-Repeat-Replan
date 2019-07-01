#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <string>

class StateVisualizer
{
public:
	visualization_msgs::Marker direct_msg, idledirect_msg;
	visualization_msgs::Marker idlejs_msg, idlectrl_msg;
	visualization_msgs::Marker js_msg, ctrlhov_msg, ctrlcmd_msg;
	visualization_msgs::Marker noodom_msg, null_msg;
	ros::Publisher led_pub;
	std::string last_type;

	StateVisualizer()
	{
		std_msgs::ColorRGBA c;
		std_msgs::ColorRGBA c1,c2;
		
		{
			c.r = 1; c.g = 1; c.b = 0; c.a = 1;
			direct_msg.colors.push_back(c);
		}
		
		{
			c.r = 1; c.g = 1; c.b = 0; c.a = 0.5;
			idledirect_msg.colors.push_back(c);
			c.r = 0; c.g = 0; c.b = 0; c.a = 0.5;
			idledirect_msg.colors.push_back(c);
		}

		{
			c.r = 0; c.g = 0; c.b = 0; c.a = 1;
			null_msg.colors.push_back(c);
		}

		{
			c.r = 0; c.g = 1; c.b = 0; c.a = 0.5;
			js_msg.colors.push_back(c);		
			c.r = 0; c.g = 0; c.b = 0; c.a = 0.5;
			js_msg.colors.push_back(c);
		}
		
		{
			c.r = 0; c.g = 1; c.b = 0; c.a = 0.5;
			idlejs_msg.colors.push_back(c);
			c.r = 1; c.g = 1; c.b = 0; c.a = 0.5;
			idlejs_msg.colors.push_back(c);
		}
		
		{
			c1.r = 0; c1.g = 1; c1.b = 0; c1.a = 0.125;
			c2.r = 0; c2.g = 0; c2.b = 0; c2.a = 0.125;
			ctrlhov_msg.colors.push_back(c1);		
			ctrlhov_msg.colors.push_back(c2);
			ctrlhov_msg.colors.push_back(c1);		
			ctrlhov_msg.colors.push_back(c2);
			c.r = 0; c.g = 0; c.b = 0; c.a = 0.5;
			ctrlhov_msg.colors.push_back(c);
		}

		{
			c1.r = 0; c1.g = 1; c1.b = 0; c1.a = 0.125;
			c2.r = 0; c2.g = 0; c2.b = 0; c2.a = 0.125;
			ctrlcmd_msg.colors.push_back(c1);		
			ctrlcmd_msg.colors.push_back(c2);
			ctrlcmd_msg.colors.push_back(c1);		
			ctrlcmd_msg.colors.push_back(c2);
			ctrlcmd_msg.colors.push_back(c1);		
			ctrlcmd_msg.colors.push_back(c2);
			ctrlcmd_msg.colors.push_back(c1);		
			ctrlcmd_msg.colors.push_back(c2);
		}

		{
			c.r = 1; c.g = 0; c.b = 0; c.a = 0.0625;
			noodom_msg.colors.push_back(c);
			c.r = 0; c.g = 0; c.b = 0; c.a = 0.0625;
			noodom_msg.colors.push_back(c);
		}
	};

	void publish_led_vis(const ros::Time& tm, const std::string& led_type)
	{
		if (led_type.compare(last_type)==0)
		{
			return;
		}

		if (led_type.compare("null")!=0 && last_type.compare("noodom")==0)
		{
			// will not exit noodom except null(turn off)
			return;
		}

		last_type = led_type;
		visualization_msgs::Marker* pMsg; 
		if (led_type.compare("direct")==0)
			pMsg = &direct_msg;
		else if (led_type.compare("idledirect")==0)
			pMsg = &idledirect_msg;
		else if (led_type.compare("idlejs")==0)
			pMsg = &idlejs_msg;
		else if (led_type.compare("js")==0)
			pMsg = &js_msg;
		else if (led_type.compare("ctrlhov")==0)
			pMsg = &ctrlhov_msg;
		else if (led_type.compare("ctrlcmd")==0)
			pMsg = &ctrlcmd_msg;
		else if (led_type.compare("noodom")==0)
			pMsg = &noodom_msg;
		else if (led_type.compare("null")==0)
			pMsg = &null_msg;
		else
			pMsg = &null_msg;
		
		pMsg->header.stamp = tm;
		led_pub.publish(*pMsg);
	};
};
