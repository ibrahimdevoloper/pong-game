import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point

from fribe.loader import load_engine_from_string

class BehaviourControlPublisher(Node):
#     behavior_description = """universe "x"
# "low" 50 0
# "high" 1100 1
# end

# universe "y"
# "low" 0 0
# "threshold" 150 0.8 
# "high" 700 1
# end

# universe "push"
# "low" 0 0.01
# "high" 1 0.025
# end

# universe "position"
# "low"  0 0.03
# "high" 1 0.19
# end

# rulebase "position"
# rule "low" when "x" is "low" end
# rule "high" when "x" is "high" end
# end

# rulebase "push"
# rule "high" when "y" is "low" end
# rule "low" when "y" is "threshold" end
# end
# """
    behavior_description = """
    universe "y"
    "low" 10 0
    "mid" 360 0.5
    "high" 720 1
    end

    universe "position"
    "low"  0 0.03
    "mid" 0.5 0.1
    "high" 1 0.19
    end
    
    universe "x"
    "low" 0 0
    "threshold" 300 0.75 
    "off" 400 0.85
    "high" 1100 1
    end

    universe "push"
    "low" 0 0.01
    "high" 1 0.025
    end

    rulebase "position"
    rule "low" when "y" is "low" end
    rule "mid" when "y" is "mid" end
    rule "high" when "y" is "high" end
    end

    rulebase "push"
    rule "low" when "x" is "low" end
    rule "high" when "x" is "threshold" end
    rule "low" when "x" is "off" end
    rule "low" when "x" is "high" end
    end
"""



    def __init__(self):
        super().__init__('behaviour_control_publisher')

        self.engine = load_engine_from_string(self.behavior_description)


        self.h_publisher_ = self.create_publisher(String, 'h_control', 10)
        self.v_publisher_ = self.create_publisher(String, 'v_control', 10)
        
        self.get_logger().info("Behaviour Control Node has started.")

        # self.create_subscription(String, '/h_position', self.h_control_callback, 1)
        # self.create_subscription(String, '/v_position', self.v_control_callback, 1)
        self.create_subscription(Point, '/position_point', self.point_callback, 1)
       
    def point_callback(self, msg):
        # Process the Point message
        x = msg.x
        y = msg.y
        
        # y is for the bat position along its access
        # Create the engine from the behavior description
        self.engine.calc_consequences({'x':x, 'y': y})
        position = self.engine.get_state('position')
        push = self.engine.get_state('push')
        # Log the values
        self.get_logger().info(f'Position: {position}, Push: {push}')
        self.get_logger().info(f'x: {x}, y: {y}')
        # Publish the control values to the respective topics
        self.h_publisher_.publish(String(data=str(push)))
        self.v_publisher_.publish(String(data=str(position)))


def main(args=None):
    rclpy.init(args=args)
    node = BehaviourControlPublisher()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()