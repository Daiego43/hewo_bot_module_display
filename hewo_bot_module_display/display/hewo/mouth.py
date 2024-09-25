import pygame
import numpy as np
from scipy.interpolate import make_interp_spline
from hewo_bot_module_display.settings.settings_loader import SettingsLoader

# Cargamos el diccionario de settings
settings = SettingsLoader().load_settings("settings.hewo")
mouth_settings = settings['mouth']


class Lip:
    def __init__(self, size, position, settings):
        self.size = size
        self.position = position
        self.color = tuple(settings['color'])
        self.lip_width = settings['lip_width']
        self.emotion = settings['emotion']  # 5-element vector: [x1, y1, center_y, x2, y2]

    def lip_shape(self):
        x_points = [
            self.clamp(self.emotion[0] / 100 * (self.size[0] / 2), 0, self.size[0] / 2),
            self.size[0] / 2,  # El centro en el eje X es fijo
            self.clamp(self.size[0] - (self.emotion[3] / 100 * (self.size[0] / 2)), self.size[0] / 2, self.size[0])
        ]

        y_points = [
            self.clamp(self.emotion[1] / 100 * self.size[1], self.lip_width, self.size[1] - self.lip_width),
            self.clamp(self.emotion[2] / 100 * self.size[1], self.lip_width, self.size[1] - self.lip_width),
            self.clamp(self.emotion[4] / 100 * self.size[1], self.lip_width, self.size[1] - self.lip_width)
        ]

        spline = make_interp_spline(x_points, y_points, k=2)
        x_range = np.linspace(min(x_points), max(x_points), 500)
        return [(int(x), int(spline(x))) for x in x_range]

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def set_emotion(self, emotion_vector):
        """
        Establece la emoción del labio usando un vector de 5 valores: [x1, y1, center_y, x2, y2].
        """
        self.emotion = emotion_vector

    def update(self):
        pass

    def draw(self, surface):
        points = self.lip_shape()
        pygame.draw.lines(surface, self.color, False, points, self.lip_width)

    def handle_event(self, event):
        pass


class Mouth:
    def __init__(self, size, position, settings=mouth_settings):
        self.size = size
        self.position = position
        self.surface = pygame.Surface(self.size)
        self.color = tuple(settings['bg_color'])

        # Creación de los labios superior e inferior utilizando los settings
        self.top_lip = Lip(self.size, self.position, settings['upper_lip'])
        self.bot_lip = Lip(self.size, self.position, settings['lower_lip'])

    def draw(self, surface):
        self.surface.fill(self.color)
        self.top_lip.draw(self.surface)
        self.bot_lip.draw(self.surface)
        surface.blit(self.surface, self.position)

    def update(self):
        self.top_lip.update()
        self.bot_lip.update()

    def handle_event(self, event):
        pass

    def set_emotion(self, top_lip_percentages, bot_lip_percentages):
        """
        Establece la emoción de los labios superior e inferior usando vectores de 5 valores.
        """
        self.top_lip.set_emotion(top_lip_percentages)
        self.bot_lip.set_emotion(bot_lip_percentages)

    def get_emotion(self):
        """
        Obtiene los vectores de emoción actuales de los labios superior e inferior.
        """
        return self.top_lip.emotion, self.bot_lip.emotion
