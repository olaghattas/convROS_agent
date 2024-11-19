import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import playsound
from google.oauth2 import service_account
from google.cloud import texttospeech
from ament_index_python.packages import get_package_share_directory

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('tts_model', "gTTS")
        self.declare_parameter('api_credential_location', "")
        self.tts_model = self.get_parameter('tts_model').get_parameter_value().string_value

        if self.tts_model == "googleAPI":
            api_credential_fileName = self.get_parameter('api_credential_file').get_parameter_value().string_value
            self.api_credential_location = os.path.join( get_package_share_directory('convros_bot'), 'config', api_credential_fileName)
            
            if self.api_credential_location == "": raise Exception("You did not provide google API Credential json file location")
            
            self.credentials = service_account.Credentials.from_service_account_file(self.api_credential_location)
            self.client = texttospeech.TextToSpeechClient(credentials=self.credentials)
            self.get_logger().warn(f'Using Google Text-to-Speech API')
        elif self.tts_model == "gTTS":
            self.get_logger().warn(f'Using gTTS for speech to text service')

        # Subscribe to the speech_text topic
        self.subscription = self.create_subscription(String, 'tts_text', self.text_to_speech_callback, 10)

    def text_to_speech_callback(self, msg):
        text = msg.data
        if text:
            # self.get_logger().info(f'Speaking: " {text}')
            if self.tts_model == "gTTS":
                self.synthesize_text_gTTS(text)
            elif self.tts_model == "googleAPI":
                self.synthesize_text_googleAPI(text)
    
    def synthesize_text_gTTS(self, text):
        # Use gTTS to generate speech and save it to an MP3 file
        tts = gTTS(text=text, lang='en', tld='us', slow=False)
        try:
            tts.save("output.mp3")

            # Play the MP3 file
            playsound.playsound("output.mp3", True)

            # Remove the MP3 file after playback
            os.remove("output.mp3")
        except Exception as e:
            self.get_logger().warn(f'Some Error Occurred!')

    def synthesize_text_googleAPI(self, text):
        """Synthesizes speech from the input string of text using google speech to text api."""
        

        input_text = texttospeech.SynthesisInput(text=text)

        # Note: the voice can also be specified by name.
        # Names of voices can be retrieved with client.list_voices().
        voice = texttospeech.VoiceSelectionParams(
            language_code="en-US",
            name="en-US-Journey-F",
            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
        )

        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )

        response = self.client.synthesize_speech(
            request={"input": input_text, "voice": voice, "audio_config": audio_config}
        )
        try:
            # The response's audio_content is binary.
            with open("output.mp3", "wb") as out:
                out.write(response.audio_content)
            
            playsound.playsound("output.mp3", True)
            # Remove the MP3 file after playback
            os.remove("output.mp3")
        except Exception as e:
            self.get_logger().warn(e)

            


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
