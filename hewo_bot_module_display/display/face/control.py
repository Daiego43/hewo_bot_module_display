import math
import pygame
import pygame_widgets
import pathlib
import yaml
from pygame_widgets.slider import Slider
from pygame_widgets.textbox import TextBox
from pygame_widgets.button import Button

emotion_dir = pathlib.Path("game/characters/hewo/face_components/emotions")


class VariableSlider:
    def __init__(self, surface, x=15, y=10, variable_name='sample', width=250, height=20, min_val=0, max_val=100,
                 step=1):
        self.surface = surface
        self.variable_name = variable_name
        self.slider = Slider(self.surface, x, y + 5, width, height, min=min_val, max=max_val, step=step, value=min_val)
        self.output = TextBox(self.surface, x + width + 20, y, 120, 30, fontSize=20)
        self.output.disable()

    def update(self):
        events = pygame.event.get()
        value = self.slider.getValue()
        self.output.setText(f'{self.variable_name}: {value}')
        pygame_widgets.update(events)
        return value


class FaceControls:
    """
    A surface that contains sliders to control variables in your game.
    """
    vars = ['letl_a', 'letl_b', 'letl_c',
            'lebl_a', 'lebl_b', 'lebl_c',
            'retl_a', 'retl_b', 'retl_c',
            'rebl_a', 'rebl_b', 'rebl_c',
            'tl_a', 'tl_b', 'tl_c', 'tl_d','tl_e',
            'bl_a', 'bl_b', 'bl_c', 'bl_d','bl_e']

    # This is bc if not the mouth would go crazy.
    win_width = 425
    win_height = 10 + 30 * len(vars) + 60

    def __init__(self, position=(0, 0)):
        pygame.init()
        self.position = position
        self.surface = pygame.Surface((self.win_width, self.win_height))
        self.values = {var: 0 for var in self.vars}  # Diccionario en vez de lista
        self.sliders = []
        y = 10
        for var in self.vars:
            slider = VariableSlider(self.surface, variable_name=var, y=y)
            slider.slider.setValue(0)
            self.sliders.append(slider)
            y += 30
        # Añadir caja de texto y botón
        self.name_input = TextBox(self.surface, 15, y + 20, 250, 30, fontSize=20)
        self.confirm_button = Button(
            self.surface, 275, y + 20, 130, 30, text='Save emotion',
            fontSize=20, margin=5,
            onClick=self.confirm_name
        )

    def confirm_name(self):
        emotion_name = self.name_input.getText()
        print(f'New emotion saved: {emotion_name}')
        with open(emotion_dir/f"{emotion_name}.yaml", "w") as emotion_file:
            yaml.dump(self.values, emotion_file)

    def update(self):
        for slider in self.sliders:
            val = slider.update()
            self.values[slider.variable_name] = val  # Guardar en el diccionario

    def draw(self, surface):
        surface.blit(self.surface, self.position)

    def handle_event(self, event):
        self.name_input.listen(event)
        self.confirm_button.listen(event)

    def get_values(self):
        return self.values

    def handle_input(self, keys, position, size, step=10):
        # Movement
        if keys[pygame.K_LEFT]:
            position[0] -= step
        if keys[pygame.K_RIGHT]:
            position[0] += step
        if keys[pygame.K_UP]:
            position[1] -= step
        if keys[pygame.K_DOWN]:
            position[1] += step

        if keys[pygame.K_HOME]:
            size[1] += step
            size[0] = size[1] * ((1 + math.sqrt(5)) / 2)

        if keys[pygame.K_END]:
            size[1] -= step
            size[0] = size[1] * ((1 + math.sqrt(5)) / 2)

        return position, size
