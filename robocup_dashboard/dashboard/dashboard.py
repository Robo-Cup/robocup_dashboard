#!env/usr/bin python3
import pygame

from robocup_dashboard.dashboard.exceptions import DashboardException
from robocup_dashboard.dashboard.key_value_list import KeyValueList
from robocup_dashboard.dashboard.key_value_grid import KeyValueGrid

WIDTH = 1000
HEIGHT = 600

KEY_VALUE_LIST_WIDTH = 200

VALUE_EDITBOX_WIDTH = 300
VALUE_EDITBOX_HEIGHT = 50

class Dashboard:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.font = pygame.font.SysFont("Arial", 20)

        self.key_value_list = KeyValueList(self.screen, (111, 45, 168), KEY_VALUE_LIST_WIDTH, HEIGHT)
        self.key_value_grid = KeyValueGrid(self.screen, KEY_VALUE_LIST_WIDTH, WIDTH-KEY_VALUE_LIST_WIDTH, HEIGHT)

        pygame.display.set_caption("Robocup Dashboard")
        
        self.keys = []
        self.values = []
        self.key_value_list_selected = -1
        self.current_text: str = ""

    def append_current_text(self, x: str) -> None:
        if self.key_value_list_selected != -1:
            self.current_text += x
    
    def pop_current_text(self) -> None:
        if self.key_value_list_selected != -1:
            self.current_text = self.current_text[:-1]

    def set_key_value_list_selected(self, x: int) -> None:
        if x == -1: return
        if x != self.key_value_list_selected:
            # self.update_value(self.key_value_list_selected, float(self.current_text))
            # Set current text value to current text so user can change it
            self.current_text = str(self.values[x])
            self.key_value_list_selected = x

    def update_value(self, i: int, value: float) -> None:
        # check if values list is long enough
        if len(self.values) <= i:
            self.values += [0.0] * (i - len(self.values) + 1)
        self.values[i] = value
        self.key_value_list.update_value(i, value)
        self.key_value_grid.update_value(i, value)

    def run(self) -> None:
        # Check events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise DashboardException()
            elif event.type == pygame.MOUSEBUTTONDOWN:

                mouse_button = pygame.mouse.get_pressed()
                if mouse_button[0]:
                    pos = pygame.mouse.get_pos()
                    if pos[0] < KEY_VALUE_LIST_WIDTH:
                        # TODO: Update value on click
                        # print("KEY VALUE LIST CLICKED")
                        self.set_key_value_list_selected(self.key_value_list.get_key_index_clicked(pos[1]))
                        self.editing_value = True
                    else:
                        # print("KEY VALUE GRID CLICKED")
                        self.key_value_grid.handle_grid_item_clicked(pos[0], pos[1])
                elif mouse_button[2]:
                    print("Right click")
                    self.key_value_list_selected = -1
                    self.editing_value = False
            
            elif event.type == pygame.KEYDOWN:
                match event.key:
                    case pygame.K_0:
                        self.append_current_text("0")
                    case pygame.K_1:
                        self.append_current_text("1")
                    case pygame.K_2:
                        self.append_current_text("2")
                    case pygame.K_3:
                        self.append_current_text("3")
                    case pygame.K_4:
                        self.append_current_text("4")
                    case pygame.K_5:
                        self.append_current_text("5")
                    case pygame.K_6:
                        self.append_current_text("6")
                    case pygame.K_7:
                        self.append_current_text("7")
                    case pygame.K_8:
                        self.append_current_text("8")
                    case pygame.K_9:
                        self.append_current_text("9")
                    case pygame.K_PERIOD:
                        if "." not in self.current_text:
                            self.append_current_text(".")
                    case pygame.K_MINUS:
                        if self.current_text == "":
                            self.append_current_text("-")
                        elif self.current_text == "0":
                            self.current_text = "-"
                    case pygame.K_BACKSPACE:
                        self.pop_current_text()
                    case pygame.K_RETURN:
                        if self.current_text != "":
                            self.update_value(self.key_value_list_selected, float(self.current_text))
                        self.key_value_list_selected = -1
                        self.current_text = ""

            
        self.key_value_list.draw(self.key_value_list_selected)
        self.key_value_grid.draw()

        # TODO: Replace black box with grid area
        # draw black rectangle over hte rest of the screen
        # pygame.draw.rect(self.screen, (0, 0, 0), pygame.Rect(KEY_VALUE_LIST_WIDTH, 0, WIDTH - KEY_VALUE_LIST_WIDTH, HEIGHT))

        # TODO: Make this a class
        # Draw current text
        if self.key_value_list_selected != -1:
            # pass
            # draw black rectangle in the middle of the screen
            pygame.draw.rect(self.screen, (25, 25, 25), pygame.Rect(WIDTH/2 - VALUE_EDITBOX_WIDTH/2 + 50, HEIGHT/2-VALUE_EDITBOX_HEIGHT/2, VALUE_EDITBOX_WIDTH, VALUE_EDITBOX_HEIGHT))
            
            key_text = self.font.render(str(self.keys[self.key_value_list_selected]), True, (255, 255, 255))
            self.screen.blit(key_text, (WIDTH/2 - VALUE_EDITBOX_WIDTH/2 + 50+10, HEIGHT/2-VALUE_EDITBOX_HEIGHT/5))

            editing_text = self.font.render(self.current_text, True, (255, 255, 255))
            self.screen.blit(editing_text, (WIDTH/2 - VALUE_EDITBOX_WIDTH/2 + 50*3+10, HEIGHT/2-VALUE_EDITBOX_HEIGHT/5))

        pygame.display.update()

    def set_keys(self, keys: list[any]) -> None:
        self.keys = keys
        if len(keys) > len(self.values):
            # Set value of any values that don't exist to 0
            self.values += [0] * (len(keys) - len(self.values))
        self.key_value_list.set_keys(keys)
        self.key_value_grid.set_keys(keys)
    
    def set_values(self, keys: list[int], values: list[float]) -> None:
        for index, key_index in enumerate(keys):
            # print(f"key_index: {key_index}, value: {values[index]}")
            self.update_value(key_index, values[index])