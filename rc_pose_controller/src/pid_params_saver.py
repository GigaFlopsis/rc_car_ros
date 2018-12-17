#!/usr/bin/env python
# coding=utf8

import yaml
import os.path

class YamlParams():
    def __init__(self, _path = os.path.dirname(__file__)+'/pid_params.yaml'):
        self.path = _path
        self.params = None
        self.params_open()
        print()


    #   Открываем файл параметров
    def params_open(self):
        if os.path.exists(self.path):
            pid_file = open(self.path, "rw")
            self.params = yaml.load(pid_file)
            # print("pid params file is opened")
        else:
            pid_file = open(self.path, "w+")
            pid_file.close()
            print("pid params file is created")

    #   Получаем параметр по ключу
    def params_get(self, key):
        value = self.params[key]
        return value

    #   Задаём параметр по ключу
    def params_set(self, key, value):
        self.params[key] = value

    #   Сохраняем все параметры в файл
    def params_save(self):
        pid_file = open(self.path, "w")
        with open(self.path, 'w') as outfile:
            yaml.dump(self.params, outfile, default_flow_style=False)
        pid_file.close()
