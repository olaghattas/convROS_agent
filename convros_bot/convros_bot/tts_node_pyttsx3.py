import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
from threading import Thread

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node_pyttsx3')

        # Initialize the TTS engine
        self.engine = pyttsx3.init()
        
        # Adjust speech properties (optional)
        self.engine.setProperty('rate', 150)  # Set speech rate (words per minute)
        self.engine.setProperty('volume', 1.0)  # Volume (0.0 to 1.0)

        # Subscribe to the speech_text topic
        self.subscription = self.create_subscription(
            String, 
            'speech_text', 
            self.text_to_speech_callback, 
            10
        )

        # Start the TTS engine loop in a separate thread to avoid blocking
        self.tts_thread = Thread(target=self._run_tts_engine)
        self.tts_thread.start()

    def text_to_speech_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Received: "{text}" - Speaking now...')
        
        # Queue the text for speaking (non-blocking)
        self.engine.say(text)
        self.engine.runAndWait()

    def _run_tts_engine(self):
        # Ensure the TTS engine loop is running continuously
        self.engine.runAndWait()

    def destroy_node(self):
        # Gracefully stop the TTS engine when the node is destroyed
        self.get_logger().info('Shutting down TTS engine...')
        self.engine.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    tts_node = TTSNode()

    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        tts_node.get_logger().info('TTS Node stopped by user.')
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
