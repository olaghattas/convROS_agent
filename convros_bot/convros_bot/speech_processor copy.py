import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
from datetime import datetime
import requests
from bs4 import BeautifulSoup
from rclpy.action import ActionClient
from convros_interfaces.action import ConversationRequest

class CentralNode(Node):
    def __init__(self):
        super().__init__('speech_processor')

        self.declare_parameter('incapability_responses', ["I'm sorry."])
        self.declare_parameter('wake_words', ["Sarah"])
        self.declare_parameter('wake_responses', ["Hello! How can I assist you today?"])
        self.declare_parameter('trigger_keywords', ["weather"])
        

        self.incapability_responses = self.get_parameter('incapability_responses').get_parameter_value().string_array_value
        self.wake_words = self.get_parameter('wake_words').get_parameter_value().string_array_value
        self.wake_responses = self.get_parameter('wake_responses').get_parameter_value().string_array_value
        self.trigger_keywords = self.get_parameter('trigger_keywords').get_parameter_value().string_array_value
        
        self.isAwake = False

        # Subscribe to the speech_text topic (from STT node)
        self.subscription = self.create_subscription(String, 'speech_text', self.speech_callback, 10)  

        # Publisher for tts_text topic (to TTS node)
        self.publisher_ = self.create_publisher(String, 'tts_text', 10)

        self.conversation_action_client = ActionClient(self, ConversationRequest, 'conversation_action')

    def speak(self, text):
        self.get_logger().info(f'Response:  {text}')
        response_msg = String()
        response_msg.data = text
        self.publisher_.publish(response_msg)

    def speech_callback(self, msg):
        # self.get_logger().info(f'Received: {msg.data}')
        speech_text = msg.data.lower()  # Convert to lowercase for easier matching

        if not self.isAwake:
            for word in self.wake_words:
                if word in speech_text:
                    self.isAwake = True
                    self.speak(random.choice(self.wake_responses))
                    is_success = self.start_conversation_action()
                    self.get_logger().info(f'is_success:  {is_success}')
                    self.isAwake = False
                    return

        # Process the text for keywords
        # self.process_speech(speech_text)
        

    def process_speech(self, text):
        if not self.isAwake:
            for word in self.wake_words:
                if word in text:
                    self.isAwake = True
                    self.speak(random.choice(self.wake_responses))
                    
        else:
            # Check for keywords in the speech text
            for keyword in self.trigger_keywords:
                if keyword in text:
                    self.execute_action(keyword)
            
            # Default response if no keyword matches
            self.speak( self.select_random_incapability_msg())
        
    def execute_action(self, keyword):
        if keyword == "time":
            # Action to announce the current time
            current_time_msg = self.get_current_time()
            self.speak(current_time_msg)
        
        elif keyword in ["weather", "way there", "temperature"]:
            # Action to greet the user
            self.speak('Hmmm... Let me check')
            weather_msg = self.get_weather()
            self.speak(weather_msg)
        
        elif keyword == "come":
            # Action to say goodbye
            self.speak("Okay let me come")

        elif keyword == "go home":
            self.speak("Ok, Thank you. I am going home.")
            self.isAwake = False
        
        elif keyword == "bye":
            # Action to say goodbye
            self.speak("Ok Bye. I am going home.")
            self.isAwake = False
        
        elif keyword in ["thanks", "thank"]:
            self.speak("You are welcome")
            self.isAwake = False
        
        elif keyword == "hear me":
            # Action to say goodbye
            self.speak("Yes I can hear you")

        else:
            # Handle unrecognized keywords
            self.speak("I'm sorry, I don't recognize that command.")

    def select_random_incapability_msg(self):
        return random.choice(self.incapability_responses)
    
    def get_current_time(self):
        current_time = datetime.now().strftime("%I:%M %p")
        message = f"The current time is {current_time}"
        return message


    def get_weather(self, location='USNH0079:1:US'):
        # Example URL for a weather site (adjust as needed)
        url = f"https://www.weather.com/weather/today/l/{location}"
        
        # Send GET request to the website
        response = requests.get(url)
        if response.status_code == 200:
            # Parse the HTML content
            soup = BeautifulSoup(response.text, 'html.parser')
            
            # Find temperature element (HTML structure may vary)
            temp_element = soup.find('span', class_='CurrentConditions--tempValue--zUBSz')
            desc_element = soup.find('div', class_='CurrentConditions--phraseValue---VS-k')
            
            if temp_element and desc_element:
                temperature = temp_element.text
                description = desc_element.text
                # print(f"Current temperature: {temperature}")
                # print(f"Weather description: {description}")
            else:
                print("Could not find weather data. The page structure might have changed.")
        else:
            print("Failed to retrieve data. Check the URL or location code.")
        response_txt = f'Current temperature is {temperature}. And its {description}.'
        print(response_txt)
        return response_txt

    def start_conversation_action(self, command="start"):
        #Either 'start' or 'stop' path recording
        goal_msg = ConversationRequest.Goal()
        goal_msg.command = command

        self.conversation_action_client.wait_for_server()
        self.get_logger().info(f'Conversation: {command}')
        return self.conversation_action_client.send_goal_async(goal_msg)
    
def main(args=None):
    rclpy.init(args=args)

    central_node = CentralNode()

    try:
        rclpy.spin(central_node)
    except KeyboardInterrupt:
        central_node.get_logger().info('Central node stopped by user.')
    finally:
        central_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
