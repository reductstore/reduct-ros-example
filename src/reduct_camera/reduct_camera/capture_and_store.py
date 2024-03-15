import asyncio
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from reduct import Bucket, BucketSettings, Client, QuotaType


class ImageListener(Node):
    """Node for listening to image messages and storing them in ReductStore."""

    def __init__(self, reduct_client: Client, loop: asyncio.AbstractEventLoop) -> None:
        """
        Initialize the image listener node.

        :param reduct_client: Client instance for interacting with ReductStore.
        :param loop: The asyncio event loop.
        """
        super().__init__("image_listener")
        self.reduct_client: Client = reduct_client
        self.loop: asyncio.AbstractEventLoop = loop
        self.bucket: Bucket = None
        self.subscription = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.get_logger().info("Image listener node initialized")


    async def init_bucket(self) -> None:
        """Asynchronously initialize the Reduct bucket for storing images."""
        self.get_logger().info("Initializing Reduct bucket")
        self.bucket = await self.reduct_client.create_bucket(
            "ros-bucket",
            BucketSettings(quota_type=QuotaType.FIFO, quota_size=1_000_000_000),
            exist_ok=True,
        )

    @staticmethod
    def display_timestamp(timestamp: int) -> str:
        """
        Format a timestamp for human-readable display.

        :param timestamp: The timestamp in microseconds.
        :return: The formatted timestamp string.
        """
        return datetime.fromtimestamp(timestamp / 1e6).isoformat()

    @staticmethod
    def get_timestamp(msg: Image) -> int:
        """
        Extract the timestamp from a ROS message.

        :param msg: The ROS message.
        :return: The timestamp in microseconds.
        """
        return int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3)
    
    @staticmethod
    def serialize_message(msg: Image) -> bytes:
        """
        Serialize a ROS message to bytes.

        :param msg: The ROS message.
        :return: The serialized message.
        """
        return bytes(msg.data)

    def image_callback(self, msg: Image) -> None:
        """
        Handle incoming image messages by scheduling storage.

        This callback is triggered by ROS message processing. It schedules
        the image storage coroutine to be executed in the asyncio event loop.
        """
        self.get_logger().info(f'Received image, storing to database')
        timestamp = self.get_timestamp(msg)
        binary_data = self.serialize_message(msg)
        asyncio.run_coroutine_threadsafe(self.store_data(timestamp, binary_data), self.loop)

    async def store_data(self, timestamp: int, data: bytes) -> None:
        """
        Store unstructured data in the Reduct bucket.

        :param timestamp: The timestamp for the data.
        :param data: The serialized data.
        """
        if not self.bucket:
            await self.init_bucket()

        self.get_logger().info(f"Storing data at {self.display_timestamp(timestamp)}")
        await self.bucket.write("image", data, timestamp)


def main() -> None:
    """
    Entry point for the application.

    Initializes the ROS node and Reduct client, starts the asyncio event loop,
    and runs ROS spinning in a separate thread.
    """
    rclpy.init()
    reduct_client = Client("http://localhost:8383")

    loop = asyncio.get_event_loop()
    image_listener = ImageListener(reduct_client, loop)
    spin_thread = threading.Thread(target=rclpy.spin, args=(image_listener,))
    spin_thread.start()

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
        spin_thread.join()
        image_listener.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
