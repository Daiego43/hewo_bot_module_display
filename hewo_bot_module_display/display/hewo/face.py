import pygame
from hewo_bot_module_display.display.hewo.eye import Eye
from hewo_bot_module_display.display.hewo.mouth import Mouth
from hewo_bot_module_display.settings.settings_loader import SettingsLoader

PHI = (1 + 5 ** (1 / 2)) / 2
settings = SettingsLoader().load_settings("settings.hewo")


class Face:
    def __init__(self, settings=None):
        if settings is None:
            self.settings = SettingsLoader().load_settings("settings.hewo")  # Si no se pasa nada, usa la configuración de 'face' en settings

        self.size = [PHI * self.settings['face']['size'],
                     self.settings['face']['size']]
        self.position = self.settings['face']['position']
        self.color = tuple(self.settings['face']['bg_color'])
        self.max_size = self.settings['face']['max_size']

        # Superficie de la cara
        self.face_surface = pygame.Surface(self.size)

        # Tamaños proporcionales de ojos y boca
        self.eye_size = [self.size[0] / 5, self.size[1] / 5 * 4]
        self.mouth_size = [self.size[0] / 5 * 3, self.size[1] / 5]

        # Posiciones de los elementos en la cara
        self.left_eye_pos = [0, 0]
        self.right_eye_pos = [self.eye_size[0] * 4, 0]
        self.mouth_pos = [self.eye_size[0], self.eye_size[1]]

        # Inicialización de los ojos y la boca usando settings correspondientes
        self.left_eye = Eye(self.eye_size, self.left_eye_pos, settings=self.settings['eye'])
        self.right_eye = Eye(self.eye_size, self.right_eye_pos, settings=self.settings['eye'])
        self.mouth = Mouth(self.mouth_size, self.mouth_pos, settings=self.settings['mouth'])

        self.set_face_elements()

    def set_face_elements(self):
        self.face_surface = pygame.Surface(self.size)
        self.eye_size = [self.size[0] / 5, self.size[1] / 5 * 4]
        self.mouth_size = [self.size[0] / 5 * 3, self.size[1] / 5]

        self.left_eye_pos = [0, 0]  # Posición en la superficie
        self.right_eye_pos = [self.eye_size[0] * 4, 0]
        self.mouth_pos = [self.eye_size[0], self.eye_size[1]]

        # Re-inicializar ojos y boca con sus posiciones y tamaños
        self.left_eye = Eye(self.eye_size, self.left_eye_pos, settings=settings['eye'])
        self.right_eye = Eye(self.eye_size, self.right_eye_pos, settings=settings['eye'])
        self.mouth = Mouth(self.mouth_size, self.mouth_pos, settings=settings['mouth'])

    def set_size(self, size):
        self.size[0] = max(PHI, min(size[0], self.max_size[0]))
        self.size[1] = max(1, min(size[1], self.max_size[1]))

    def set_position(self, pos):
        self.position[0] = max(0, min(pos[0], self.max_size[0] - self.size[0]))
        self.position[1] = max(0, min(pos[1], self.max_size[1] - self.size[1]))

    def update(self):
        self.left_eye.update()
        self.right_eye.update()
        self.mouth.update()
        self.handle_input()
        self.update_emotion()

    def handle_event(self, event):
        self.left_eye.handle_event(event)
        self.right_eye.handle_event(event)
        self.mouth.handle_event(event)

    def draw(self, surface):
        self.face_surface.fill(self.color)
        self.left_eye.draw(self.face_surface)
        self.right_eye.draw(self.face_surface)
        self.mouth.draw(self.face_surface)
        surface.blit(self.face_surface, dest=self.position)

    def get_emotion(self):
        letl = self.left_eye.top_lash.get_emotion()
        lebl = self.left_eye.bot_lash.get_emotion()
        retl = self.right_eye.top_lash.get_emotion()
        rebl = self.right_eye.bot_lash.get_emotion()
        tl, bl = self.mouth.get_emotion()
        emotions = [letl, lebl, retl, rebl, tl, bl]
        emotions = [int(item) for sublist in emotions for item in sublist]
        return emotions

    def set_emotions(self, edict):
        letl = [edict['letl_a'], edict['letl_b'], edict['letl_c']]
        lebl = [edict['lebl_a'], edict['lebl_b'], edict['lebl_c']]
        retl = [edict['retl_a'], edict['retl_b'], edict['retl_c']]
        rebl = [edict['rebl_a'], edict['rebl_b'], edict['rebl_c']]
        tl = [edict['tl_a'], edict['tl_b'], edict['tl_c'], edict['tl_d'], edict['tl_e']]
        bl = [edict['bl_a'], edict['bl_b'], edict['bl_c'], edict['bl_d'], edict['bl_e']]
        self.left_eye.set_emotion(letl, lebl)
        self.right_eye.set_emotion(retl, rebl)
        self.mouth.set_emotion(tl, bl)

    def update_emotion(self):
        pass

    def handle_input(self):
        keys = pygame.key.get_pressed()
        self.set_face_elements()


# Surface Effects (opcional, como estaba en el código original)
def pixelate(surface, pixels_factor=128, size=None):
    surface = pygame.transform.scale(surface, [PHI * pixels_factor, pixels_factor])
    surface = pygame.transform.scale(surface, size)
    return surface


if __name__ == '__main__':
    pygame.init()
    settings = SettingsLoader().load_settings("settings.hewo")
    print(settings)
    screen = pygame.display.set_mode((800, 600))
    face = Face()
    clock = pygame.time.Clock()

    while True:
        screen.fill((255, 255, 255))
        face.update()
        face.draw(screen)
        pygame.display.flip()
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            face.handle_event(event)