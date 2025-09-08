import sys
import rclpy
from rclpy.node import Node

from memory_service_interfaces.srv import UpdateMemory, GetMemory


class MemoryClient(Node):
    def __init__(self):
        super().__init__('memory_client')
        self.update_client = self.create_client(UpdateMemory, 'update_memory')
        self.get_client = self.create_client(GetMemory, 'get_memory')

        while not self.update_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for update_memory service...')
        while not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_memory service...')

    def send_update_request(self, user_input, explanation):
        req = UpdateMemory.Request()
        req.user_input = user_input
        req.explanation = explanation
        future = self.update_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_get_request(self):
        req = GetMemory.Request()
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = MemoryClient()

    # Example usage
    update_response = client.send_update_request('ciao voglio mangiare la nutella', ['q1', 'q2'], 'result1, result2', 'puoi mangiare la nutella')
    print('Updated list:', update_response.memory_list)

    get_response = client.send_get_request()
    print('Current Memory:', get_response.memory_list)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
