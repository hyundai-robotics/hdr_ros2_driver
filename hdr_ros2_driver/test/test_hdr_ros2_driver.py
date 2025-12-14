import sys
import time
from importlib import import_module

import rclpy
from rclpy.node import Node

from hdr_service_common import get_category_services, fill_request_fields


class ServiceCaller(Node):
    def __init__(self):
        super().__init__('hdr_service_tester')

    def call_service(self, service_name, service_type_str):
        pkg_name, _, srv_name = service_type_str.split('/')
        module = import_module(f'{pkg_name}.srv')
        srv_class = getattr(module, srv_name)

        client = self.create_client(srv_class, service_name)

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Service {service_name} not available.")
            return False, 'timeout'

        request = srv_class.Request()
        fill_request_fields(service_type_str, request, service_name)
        self.get_logger().info(f"Request: {request}")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            if hasattr(result, 'success') and result.success is False:
                return False, str(result)
            return True, str(result)
        else:
            return False, str(future.exception())


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_hdr_ros2_driver.py [all|version|project|control|robot|plc|etc|file|task|console]")
        return

    category = sys.argv[1]
    service_map = get_category_services(category)

    if not service_map:
        print(f"No services found for category '{category}'")
        return

    rclpy.init()
    node = ServiceCaller()

    total = len(service_map)
    success_count = 0
    failure_count = 0

    for service, srv_type in service_map:
        node.get_logger().info(f"Calling {service} [{srv_type}] ...")
        success, result = node.call_service(service, srv_type)
        if success:
            node.get_logger().info(f"Service \"{service}\" successed: {result}")
            success_count += 1
        else:
            node.get_logger().error(f"Service \"{service}\" failed: {result}")
            failure_count += 1
        time.sleep(1.0)

    node.get_logger().info("Test Summary -------------------------------")
    node.get_logger().info(f"Total Services Tried : {total}")
    node.get_logger().info(f"Success Count        : {success_count}")
    node.get_logger().info(f"Failure Count        : {failure_count}")
    node.get_logger().info("--------------------------------------------")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
