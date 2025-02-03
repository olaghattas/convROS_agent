# convROS_agent
Add conversational capability in ROS bots

## Installation
```
sudo apt-get update
sudo apt-get install python3-dev
sudo apt-get install portaudio19-dev

pip3 install pyaudio

pip3 install RealtimeSTT

pip install google-auth
pip install google-cloud-texttospeech

```

## Usage
**To launch the action server**
```
ros2 launch convros_bot question_response_action.launch.py
```

**Exapmple Action call**
```
ros2 action send_goal --feedback question_response_action convros_interfaces/action/QuestionResponseRequest "{question: 'Do you want me to show how to operate the Microwave oven?'}"
```
