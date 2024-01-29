#!/usr/bin/env python3
import PySimpleGUI as sg
from PIL import Image, ImageTk
import cv2


BORDER_COLOR = '#C7D5E0'
DARK_HEADER_COLOR = '#1B2838'
BPAD_TOP = ((20, 20), (20, 10))
BPAD_LEFT = ((20, 10), (0, 0))
BPAD_LEFT_INSIDE = (0, (10, 0))
BPAD_RIGHT = ((10, 20), (10, 0))


class ImageViewer:
    def __init__(self, img_size=(800, 400)):
        self.graph = sg.Image(size=img_size, pad=(20, 20), key='-IMAGE-')
        self.img_size = img_size

    def show_img(self, img):
        resized_img = cv2.resize(img, self.img_size,
                                 interpolation=cv2.INTER_AREA)

        return ImageTk.PhotoImage(Image.fromarray(resized_img))


class HMI:
    def __init__(self, img_sz) -> None:

        theme_dict = {'BACKGROUND': '#2B475D',
                      'TEXT': '#FFFFFF',
                      'INPUT': '#F2EFE8',
                      'TEXT_INPUT': '#000000',
                      'SCROLL': '#F2EFE8',
                      'BUTTON': ('#000000', '#C2D4D8'),
                      'PROGRESS': ('#FFFFFF', '#C7D5E0'),
                      'BORDER': 0, 'SLIDER_DEPTH': 0, 'PROGRESS_DEPTH': 0}

        sg.theme_add_new('Dashboard', theme_dict)
        sg.theme('Dashboard')

        self.camera_img_viewer = ImageViewer(img_sz)
        self.window = self._init_layout()

    def _init_layout(self):
        top_banner = [
            [sg.Text('Titan Farm', font='Any 20', background_color=DARK_HEADER_COLOR, enable_events=True, grab=False), sg.Push(background_color=DARK_HEADER_COLOR),
             sg.Text('Friday 5 Jan 2024', font='Any 20', background_color=DARK_HEADER_COLOR)],
        ]

        col1 = [[self.LEDIndicator('_wait_')], [sg.Text('Waiting')]]
        col2 = [[self.LEDIndicator('_run_')], [sg.Text('Running')]]
        col3 = [[self.LEDIndicator('_complete_')], [sg.Text('Completed')]]
        top = [[sg.Push(), sg.Text('The Status Of The Taskh', font='Any 20'), sg.Push()],
               [sg.Push(), sg.Column(col1, element_justification='c'), sg.Push(),
                sg.Column(col2, element_justification='c'), sg.Push(),
                sg.Column(col3, element_justification='c'), sg.Push()]]

        block_3 = [[sg.Text('Package Type', font='Any 20')],
                   [sg.Radio("Type1", "bag", key='Type1', font='Any 12',
                             enable_events=True, default=True, size=(10, 1)),
                    sg.Image("figures/type1.png", enable_events=True)],
                   [sg.Radio("Type2", "bag", key='Type2', font='Any 12',
                             enable_events=True, size=(10, 1)),
                    sg.Image("figures/type2.png", enable_events=True)],
                   [sg.Radio("Type3", "bag", key='Type3', font='Any 12',
                             enable_events=True, size=(10, 1)),
                    sg.Image("figures/type3.png", enable_events=True)],
                   [sg.T('Select the type of the packages.', font='Any 12')],
                   [sg.Button('Go'), sg.Button('Exit')]]

        block_4 = [[sg.Text('', font='Any 20')],
                   [sg.Push(), sg.Text('Real-time scene', font='Any 20'), sg.Push()],
                   [sg.Text('', font='Any 20')],
                   [sg.Text('', font='Any 20')],
                   [self.camera_img_viewer.graph]]

        layout = [
            [sg.Frame('', top_banner, pad=(0, 0), background_color=DARK_HEADER_COLOR,
                      expand_x=True, border_width=0, grab=True)],
            [sg.Frame('', top, size=(1820, 200), pad=BPAD_TOP, expand_x=True,
                      relief=sg.RELIEF_GROOVE, border_width=3)],
            [sg.Frame('', [[sg.Frame('', block_3, size=(900, 1000), pad=BPAD_LEFT_INSIDE, border_width=0, expand_x=True, expand_y=True,
                                     element_justification='c')]],
                      size=(900, 1000), pad=BPAD_LEFT, background_color=BORDER_COLOR, border_width=0, expand_x=True, expand_y=True),
             sg.Column(block_4, size=(900, 1000), pad=BPAD_RIGHT, expand_x=True, expand_y=True, grab=True, element_justification='c'),],
            [sg.Sizegrip(background_color=BORDER_COLOR)]]

        window = sg.Window('Dashboard PySimpleGUI-Style', layout, margins=(0, 0), background_color=BORDER_COLOR,
                           no_titlebar=True, resizable=True, finalize=True, right_click_menu=sg.MENU_RIGHT_CLICK_EDITME_VER_LOC_EXIT)
        return window

    def SetLED(self, window, key, color):
        graph = window[key]
        graph.erase()
        graph.draw_circle((0, 0), 30, fill_color=color, line_color=color)

    def WaitingLED(self):
        self.SetLED(self.window, '_wait_', 'green')
        self.SetLED(self.window, '_run_', 'red')
        self.SetLED(self.window, '_complete_', 'red')

    def RunningLED(self):
        self.SetLED(self.window, '_wait_', 'red')
        self.SetLED(self.window, '_run_', 'green')
        self.SetLED(self.window, '_complete_', 'red')

    def CompletedLED(self):
        self.SetLED(self.window, '_wait_', 'red')
        self.SetLED(self.window, '_run_', 'red')
        self.SetLED(self.window, '_complete_', 'green')

    def LEDIndicator(self, key=None, radius=50):
        return sg.Graph(canvas_size=(radius, radius),
                        graph_bottom_left=(-radius, -radius),
                        graph_top_right=(radius, radius),
                        pad=(0, 0), key=key)

    def update_camera_img(self, rgb_img):
        self.window['-IMAGE-'].update(
            data=self.camera_img_viewer.show_img(rgb_img))

    def quit(self):
        self.window.close()
