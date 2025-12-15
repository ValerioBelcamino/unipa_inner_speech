import rclpy
from rclpy.node import Node

from memory_service_interfaces.srv import UpdateMemory, GetMemory

from memory_service.memory_manager_llm import MemoryAgent


class MemoryServer(Node):
    def __init__(self):
        super().__init__('memory_server')

        self.memory_agent = MemoryAgent()

        # Create the two services
        self.update_service = self.create_service(UpdateMemory, 'update_memory', self.update_memory_callback)
        self.get_service = self.create_service(GetMemory, 'get_memory', self.get_memory_callback)

    def update_memory_callback(self, request, response):
        self.get_logger().info(f"UpdateMemory request: user_input={request.user_input}, response={request.explanation}")

        try:
            # Append messages
            self.memory_agent.append_message(request.user_input, 'user')
            self.memory_agent.append_message(request.explanation, 'assistant')
            
            # Run the agent and get the state dict
            state = self.memory_agent.run_memory_agent(interaction_mode='insert')
            
            # Extract core_memory and populate the response object
            core_memory = state["core_memory"]
            response.memory_list = core_memory
            
            self.get_logger().info(f"Core memory: {response.memory_list}")
            
            # Return the response object
            return response
            
        except Exception as e:
            self.get_logger().error(f"UpdateMemory error: {e}")
            response.memory_list = []
            return response

    def get_memory_callback(self, request, response):
        self.get_logger().info("GetMemory request")
        
        try:
            # Run the agent in retrieve mode
            state = self.memory_agent.run_memory_agent(interaction_mode='retrieve')
            
            # Extract core memory and populate the response
            core_memory = state["core_memory"]
            
            
            # Extract last messages and populate the response object
            messages = [message.content for message in state["messages"]]
            response.last_messages = messages
            
            self.get_logger().info(f"Last messages: {response.last_messages}")
            self.get_logger().info(f"Returning memory_list: {core_memory}")
            
            response.memory_list = core_memory
            response.last_messages = messages
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"GetMemory error: {e}")
            response.memory_list = []
            return response


def main(args=None):
    rclpy.init(args=args)
    memory_server = MemoryServer()
    rclpy.spin(memory_server)
    memory_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
