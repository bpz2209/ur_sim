'''
Description: 
Version: 1.0
Autor: Julian Lin
Date: 2024-09-19 23:57:12
LastEditors: Julian Lin
LastEditTime: 2024-09-21 15:08:45
'''
import OpenAI
import datetime
import json
import os
import base64

def generate_system_prompt(json_input, color_image, depth_image):
    system_prompt = f"""
    You will be provided with a JSON input, a color image (RGB), and a depth image. Analyze the color image to recognize a specific object and use the depth image to estimate its 3D pose (position and orientation). The JSON input provides additional context or details about the object. Use the information from both the color image, depth image, and JSON data to produce the following output in JSON format:

    - Object label: The recognized object's name or category from the image and JSON input.
    - Description: A brief description of the recognized object based on both the color image and JSON input.
    - Position: The 3D coordinates of the object (x, y, z) from the depth image.
    - Orientation: The rotation of the object (roll, pitch, yaw) based on the depth image.

    Example Input:
    JSON: {{
        "object_label": "chair",
        "description": "A wooden chair with a cushioned seat."
    }}
    color image: [color image input]
    depth image: [depth image input]

    Provide the output in the following JSON format:

    {{
    "object_label": "chair",
    "description": "A wooden chair with a cushioned seat.",
    "position": {{
        "x": value_from_depth_image,
        "y": value_from_depth_image,
        "z": value_from_depth_image
    }},
    "orientation": {{
        "roll": value_from_depth_image,
        "pitch": value_from_depth_image,
        "yaw": value_from_depth_image
    }}
    }}

    Example Output:
    {{
    "object_label": "chair",
    "description": "A wooden chair with a cushioned seat.",
    "position": {{
        "x": 1.23,
        "y": 0.56,
        "z": 0.78
    }},
    "orientation": {{
        "roll": 0.02,
        "pitch": 0.15,
        "yaw": 0.10
    }}
    }}
    """
    return system_prompt


class LLM:
    def __init__(self, api_key_, url_, model_name_):
        self.api_key = api_key_
        self.url = url_
        self.model_name = model_name_
        self.max_tokens = 300
        self.client = OpenAI(api_key=self.api_key, url=self.url)
        self.chat_history = []
        self.log_file_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".json"
        log_dir = "logs"
        self.log_file_path = os.path.join(log_dir, self.log_file_name)
        self.image_log_file_path = os.path.join(log_dir, "images")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        if not os.path.exists(self.image_log_file_path):
            os.makedirs(self.image_log_file_path)
    def chat(self, role_, content_):
        message = {
            "role": role_,
            "content": content_
        }
        self.chat_history.append(message)
        completion = self.client.complete(model=self.model_name, messages=self.chat_history)
        response = completion.choices[0].message
        self.chat_history.append({
            "role": "assistant",
            "content": response
        })
        # save log in end of conversation
        with open(self.log_file_name, "w") as f:
            json.dump(self.chat_history, f)
        return response
    
    def set_system_prompt(self):
        system_prompt = generate_system_prompt()
        self.chat("system", system_prompt)

    def encode_image(self, image_):
        image_base64 = base64.b64encode(image_).decode("utf-8")
        return image_base64

    def chat_with_image(self, role_, content_, image_, depth_image_):
        input_text = {"role": role_, "type": "text", "text": content_}
        input_color_text = {"role": role_, "type": "text", "text": "color image:"}
        input_color_image = {"role": role_, "type": "image_url", "image_url": {"url": f"data:image/png;base64,{self.encode_image(image_)}"}}
        input_depth_text = {"role": role_, "type": "text", "text": "depth image:"}
        input_depth_image = {"role": role_, "type": "image_url", "image_url": {"url": f"data:image/png;base64,{self.encode_image(depth_image_)}"}}
        content = [input_text, input_color_text, input_color_image, input_depth_text, input_depth_image]
        message = {
            "role": role_,
            "content": content,
        }
        self.chat_history.append(message)
        completion = self.client.complete(model=self.model_name, messages=self.chat_history, max_tokens=self.max_tokens)
        response_message = completion.choices[0].message
        response = response_message["content"]
        self.chat_history.append({
            "role": "assistant",
            "content": response
        })
        # save log in end of conversation
        with open(self.log_file_name, "w") as f:
            json.dump(self.chat_history, f)
        # save image
        # count color image and depth image
        color_img_list = [img for img in os.listdir(self.image_log_file_path) if "color_image" in img]
        img_name = str(len(color_img_list)) + "_color_image.png"
        img_path = os.path.join(self.image_log_file_path, img_name)
        with open(img_path, "wb") as f:
            f.write(image_)
        depth_img_list = [img for img in os.listdir(self.image_log_file_path) if "depth_image" in img]
        depth_img_name = str(len(depth_img_list)) + "_depth_image.png"
        depth_img_path = os.path.join(self.image_log_file_path, depth_img_name)
        with open(depth_img_path, "wb") as f:
            f.write(depth_image_)
        return response
        