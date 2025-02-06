import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast


class User_Input_Keyboard(Node):

    def __init__(self):
        super().__init__('user_input_keyboard')
        self.pub_user_input = self.create_publisher(String, '/user_input', 10)
        self.sub_user_input_activation = self.create_subscription(
            String,
            '/user_input_activation',
            self.user_input_activation_callback,
            10
        )

    def user_input_activation_callback(self, msg):
        self.get_logger().info(f'Received activation message: {msg.data}')
        msg_dict = ast.literal_eval(msg.data)
        input_dialogue = msg_dict['response'] + '\n'
        old_request = msg_dict['question']
        # print("\033[34m" + f'{input_dialogue},\n {old_request}' + "\033[0m")

        self.send_msg(input_dialogue, old_request)



    def send_msg(self, input_dialogue='Ciao, come posso aiutarti?\n', old_request=''):
        msg = String()
        print("\033[34m" + f'{input_dialogue}' + "\033[0m", end='')
        msg_text = input()
        msg_text = old_request + msg_text
        msg.data = msg_text

        self.pub_user_input.publish(msg)
        self.get_logger().info(f'Published message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    uik = User_Input_Keyboard()

    uik.send_msg()
    rclpy.spin(uik)


    uik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()