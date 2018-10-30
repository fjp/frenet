#!/bin/bash

echo Hello world!

pyside2-uic -x -o ui/frenet_main_window_ui.py ui/frenet_main_window.ui

python main.py
