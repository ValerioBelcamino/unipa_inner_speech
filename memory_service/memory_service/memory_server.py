import rclpy
from rclpy.node import Node

from memory_service_interfaces.srv import UpdateMemory, GetMemory

from memory_service.memory_manager_llm import MemoryAgent  # Import your LLM wrapper class


class MemoryServer(Node):
    def __init__(self):
        super().__init__('memory_server')

        self.memory_agent = MemoryAgent()

        # Create the two services
        self.update_service = self.create_service(UpdateMemory, 'update_memory', self.update_memory_callback)
        self.get_service = self.create_service(GetMemory, 'get_memory', self.get_memory_callback)

    def update_memory_callback(self, request, response):
        self.get_logger().info(f"UpdateMemory request: user_input={request.user_input}, response={request.response}")

        # Prepare inputs for the LLM: current memory + new info from request
        # Here, you pass the current memory list and new update info to the LLM
        self.memory_agent.append_message(request, 'user')
        self.memory_agent.append_message(response, 'assistant')
        return self.memory_agent.run_memory_agent(interaction_mode='insert')

    def get_memory_callback(self, request, response):
        self.get_logger().info("GetMemory request")
        return self.memory_agent.run_memory_agent(interaction_mode='retrieve')


def main(args=None):
    rclpy.init(args=args)
    memory_server = MemoryServer()
    rclpy.spin(memory_server)
    memory_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
