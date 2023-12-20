#!env/usr/bin python3
import pygame

HEIGHT_BETWEEN_KEYS = 50
KEY_VALUE_OFFSET_FROM_TOP = 15
KEY_OFFSET_FROM_EDGE = 10
VALUE_OFFSET_FROM_WIDTH = 75

class KeyValueList:
    def __init__(self, screen: pygame.Surface, color: pygame.Color, width: int, height: int):
        self.screen = screen
        self.color = color
        self.width = width
        self.height = height
        self.rect = pygame.Rect(0, 0, self.width, self.height)

        self.keys = []
        self.values = []

        self.font = pygame.font.SysFont("Arial", 20)


    def draw(self, x: int) -> None:
        pygame.draw.rect(self.screen, self.color, self.rect)
        
        # Draw keys + values
        for i in range(len(self.keys)):
            # Draw rectangle border around key value pair
            border_color = (255, 255, 255)
            border_width = 1
            if i == x:
                border_color = (0, 255, 0)
                border_width = 3
            pygame.draw.rect(self.screen, border_color, pygame.Rect(0, i * HEIGHT_BETWEEN_KEYS, self.width, HEIGHT_BETWEEN_KEYS), border_width)

            key = self.keys[i]
            text = self.font.render(str(key), True, (255, 255, 255))
            self.screen.blit(text, (KEY_OFFSET_FROM_EDGE, KEY_VALUE_OFFSET_FROM_TOP + i * HEIGHT_BETWEEN_KEYS))

            if i < len(self.values):
                value = self.values[i]
            else:
                value = "NULL"
            text = self.font.render(str(value), True, (255, 255, 255))
            self.screen.blit(text, (self.width-VALUE_OFFSET_FROM_WIDTH, KEY_VALUE_OFFSET_FROM_TOP + i * HEIGHT_BETWEEN_KEYS))
        
    def set_keys(self, keys: list[any]) -> None:
        self.keys = keys

        if len(keys) > len(self.values):
            # Set value of any values that don't exist to "NULL"
            self.values += ["NULL"] * (len(keys) - len(self.values))

    def update_value(self, index: int, value: any) -> None:
        self.values[index] = value
    
    def get_key_index_clicked(self, y: int) -> int:
        # Calculate which key was pressed based on y position
        retval = y // HEIGHT_BETWEEN_KEYS
        if retval < len(self.keys):
            return retval
        else:
            return -1