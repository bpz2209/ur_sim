'''
Description: 
Version: 1.0
Autor: Julian Lin
Date: 2024-09-19 23:57:12
LastEditors: Julian Lin
LastEditTime: 2024-09-20 00:09:23
'''
import OpenAI
import datetime
import json
import os
class LLM:
    def __init__(self, api_key_, url_, model_name_):
        self.api_key = api_key_
        self.url = url_
        self.model_name = model_name_
        self.client = OpenAI(api_key=self.api_key, url=self.url)
        self.chat_history = []
        self.log_file_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".json"
        log_dir = "logs"
        self.log_file_path = os.path.join(log_dir, self.log_file_name)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
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
        