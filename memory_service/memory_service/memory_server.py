import rclpy
from rclpy.node import Node

from memory_service_interfaces.srv import UpdateMemory, GetMemory

from memory_service.memory_manager_llm import MemoryManagerLLM  # Import your LLM wrapper class


class MemoryServer(Node):
    def __init__(self):
        super().__init__('memory_server')

        # Initialize memory list as a list of strings (facts)
        self.memory_list = []

        # Initialize the MemoryManagerLLM instance
        self.memory_llm = MemoryManagerLLM(node_name='memory_server')

        # Create the two services
        self.update_service = self.create_service(UpdateMemory, 'update_memory', self.update_memory_callback)
        self.get_service = self.create_service(GetMemory, 'get_memory', self.get_memory_callback)

    def update_memory_callback(self, request, response):
        self.get_logger().info(f"UpdateMemory request: user_input={request.user_input}, queries={request.queries}, results={request.results}")

        # Prepare inputs for the LLM: current memory + new info from request
        # Here, you pass the current memory list and new update info to the LLM
        updated_memory_list = self.memory_llm.get_LLM_response(
            current_memory=self.memory_list,
            user_input=request.user_input,
            queries=request.queries,
            results=request.results,
            explanation=request.explanation
        )

        # Update the internal memory list with the LLM response
        self.memory_list = updated_memory_list

        # Return the updated memory list in the service response
        response.memory_list = self.memory_list
        return response

    def get_memory_callback(self, request, response):
        self.get_logger().info("GetMemory request")
        response.memory_list = self.memory_list
        return response


def main(args=None):
    rclpy.init(args=args)
    memory_server = MemoryServer()
    rclpy.spin(memory_server)
    memory_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
