import pygame
import numpy as np
from scipy.interpolate import make_interp_spline
from hewo_bot_module_display.settings.settings_loader import SettingsLoader

settings = SettingsLoader().load_settings("settings.hewo")

eye_settings = settings['elements']['left_eye']['elements']


class Pupil:
    pupil = eye_settings["pupil"]
    COLOR = (pupil["color"]["r"],
             pupil["color"]["g"],
             pupil["color"]["b"])

    def __init__(self, size, position, color=COLOR):
        self.size = size
        self.position = position
        self.color = color

    def update(self):
        pass

    def set_size(self, size):
        self.size = size

    def set_position(self, position):
        self.position = position

    def handle_event(self, event):
        pass

    def draw(self, surface):
        pygame.draw.ellipse(surface, self.color, (0, 0, self.size[0], self.size[1]))


class EyeLash:
    lash_settings = eye_settings['top_lash']
    COLOR = (
        lash_settings['color']['r'],
        lash_settings['color']['g'],
        lash_settings['color']['b']
    )

    def __init__(self, size, position, color=COLOR, init_pcts=[0, 0, 0], flip=False):
        self.size = size
        self.position = position
        self.color = color
        self.max_emotion = self.size[1]
        self.emotion_pcts = init_pcts
        x, y = position
        w, h = size
        self.polygon_points = [
            [0 + x, 0 + y],
            [0 + x, h + y],
            [w / 2 + x, h + y],
            [w + x, h + y],
            [w + x, 0 + y],
            [w / 2 + x, 0 + y]
        ]
        self.flip = flip
        self.set_points_by_pct(init_pcts)

    def handle_event(self, event):
        pass

    def update(self):
        pass

    def set_points_by_pct(self, emotion):
        self.set_emotion_pcts(emotion)
        indices = [1, 2, 3]
        if self.flip:
            self.emotion_pcts = [100 - e for e in self.emotion_pcts]
            indices = [0, 5, 4]

        for i, tup in enumerate(zip(indices, self.emotion_pcts)):
            self.polygon_points[tup[0]][1] = self.position[1] + self.size[1] * (tup[1] / 100)

    def draw(self, surface):
        points = self.polygon_points[1:4]
        if self.flip:
            points = [self.polygon_points[0], self.polygon_points[5], self.polygon_points[4]]
        ############################
        x_points = np.array([p[0] for p in points])
        y_points = np.array([p[1] for p in points])
        spline = make_interp_spline(x_points, y_points, k=2)
        x_range = np.linspace(min(x_points), max(x_points), 500)
        interpolated_points = [(int(x), int(spline(x))) for x in x_range]
        ############################
        polygon = [self.polygon_points[0]] + interpolated_points + self.polygon_points[4:]
        if self.flip:
            interpolated_points.reverse()
            polygon = self.polygon_points[1:4] + interpolated_points
        pygame.draw.polygon(surface, self.color, polygon)

    def set_emotion_pcts(self, emotion):
        for i, e in enumerate(emotion):
            self.emotion_pcts[i] = max(0, min(e, 100))

    def get_emotion(self):
        return self.emotion_pcts

    def set_emotion(self, emotion):
        self.set_emotion_pcts(emotion)
        self.set_points_by_pct(emotion)


class Eye:
    surface = settings['elements']['left_eye']['surface']
    SURFACE_COLOR = (
        surface['color']['r'],
        surface['color']['g'],
        surface['color']['b']
    )

    def __init__(self, size, position, init_emotion=None):
        self.size = size
        self.position = position
        self.lash_size = (self.size[0], self.size[1] / 2)
        self.t_pos = (0, 0)
        self.b_pos = (0, self.size[1] / 2)
        if init_emotion is None:
            self.t_emotion = [0, 0, 0]
            self.b_emotion = [0, 0, 0]
        else:
            self.t_emotion = init_emotion[0]
            self.b_emotion = init_emotion[1]
        self.top_lash = EyeLash(size=self.lash_size, position=self.t_pos)
        self.pupil = Pupil(size=self.size, position=self.position)
        self.bot_lash = EyeLash(size=self.lash_size, position=self.b_pos, flip=True)

        self.eye_surface = pygame.Surface(self.size)

    def handle_event(self, event):
        self.top_lash.handle_event(event)
        self.pupil.handle_event(event)
        self.bot_lash.handle_event(event)

    def draw(self, surface):
        self.eye_surface = pygame.surface.Surface(self.size)
        self.eye_surface.fill(self.SURFACE_COLOR)
        self.pupil.draw(self.eye_surface)
        self.top_lash.draw(self.eye_surface)
        self.bot_lash.draw(self.eye_surface)
        surface.blit(self.eye_surface, self.position)

    def update(self):
        self.top_lash.update()
        self.pupil.update()
        self.bot_lash.update()

    def set_emotion(self, t_emotion, b_emotion):
        self.top_lash.set_emotion(t_emotion)
        self.bot_lash.set_emotion(b_emotion)

    def get_emotion(self):
        return self.top_lash.get_emotion(), self.bot_lash.get_emotion()
