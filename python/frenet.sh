#!/bin/bash

echo Frenet GUI!

pyside2-uic -x -d -o gui/ui/frenet_main_window_ui.py gui/ui/frenet_main_window.ui

python main.py
