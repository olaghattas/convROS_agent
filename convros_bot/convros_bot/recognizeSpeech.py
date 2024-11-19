from RealtimeSTT import AudioToTextRecorder
# import pyautogui

def process_text(text):
    print(text)

if __name__ == '__main__':
    print("Wait until it says 'speak now'")
    recorder = AudioToTextRecorder(model = 'base.en', input_device_index=0, spinner=False, min_gap_between_recordings=1.0,)

    while True:
        recorder.text(process_text)

# import requests
# from bs4 import BeautifulSoup

# def get_current_temperature():
#     url = 'https://www.google.com/search?q=current+weather+description'
    
#     # Send a GET request to the website
#     response = requests.get(url)
    
#     # Parse the page content using BeautifulSoup
#     soup = BeautifulSoup(response.text, 'html.parser')
#     print(soup)
#     # Look for the <div> containing the temperature with class 'heading'
#     temp_div = soup.find('div', class_='display-temp')
#     print(temp_div)
    
#     if temp_div:
#         current_temp = temp_div.text.strip()
#         return current_temp
#     else:
#         return "Temperature section not found."

# # Example usage
# temperature = get_current_temperature()
# print(temperature)
