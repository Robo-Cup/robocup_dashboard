#!env/usr/bin python3
import pygame

ROWS = 3
COLS = 4

class KeyValueGrid:
    def __init__(self, screen: pygame.Surface, key_value_list_width: int, width: int, height: int):
        self.screen = screen
        self.key_value_list_width = key_value_list_width
        self.width = width
        self.height = height

        self.keys = []
        self.values = []

        self.font = pygame.font.SysFont("Arial", 20)

        self.grid_items = []
        for r in range(ROWS):
            for c in range(COLS):
                self.grid_items.append(KeyValueGridItem(screen, r, c, key_value_list_width, self.width / COLS, self.height / ROWS))
    
    def draw(self) -> None:
        # draw grid items
        grid_item_moving = None
        for grid_item in self.grid_items:
            if grid_item.moving:
                grid_item_moving = grid_item
            else:
                grid_item.draw(self.get_item_clicked)
        if grid_item_moving is not None:
            grid_item_moving.draw_fake_while_moving()
            grid_item_moving.draw(self.get_item_clicked)
        
    def get_item_clicked(self, x: int, y: int) -> 'KeyValueGridItem':
        for grid_item in self.grid_items:
            if grid_item.x < x < grid_item.x + grid_item.width and grid_item.y < y < grid_item.y + grid_item.height:
                return grid_item

    def handle_grid_item_clicked(self, x: int, y: int) -> None:
        grid_item = self.get_item_clicked(x, y)
        if grid_item is not None:
            grid_item.moving = True

    def set_keys(self, keys: list) -> None:
        if len(keys) > len(self.values):
            # Set value of any values that don't exist to "NULL"
            self.values += [0.0] * (len(keys) - len(self.values))
        
        # Update grid items
        for index, key in enumerate(keys):
            if len(self.keys) <= index:
                self.keys.append(key)
                self.grid_items[index].key = key
                self.grid_items[index].value = self.values[index]
            elif self.keys[index] != key:
                self.keys[index] = key
                self.grid_items[index].key = key
                self.grid_items[index].value = self.values[index]
    
    def update_value(self, index: int, value: float) -> None:
        # Check if values list is long enough
        if len(self.values) <= index:
            self.values += [0.0] * (index - len(self.values) + 1)
        self.values[index] = value
        self.grid_items[index].value = value

GRID_ITEM_BACKGROUND = (34,34,34)
GRID_ITEM_BORDER = (97, 9, 107)

class KeyValueGridItem:
    moving = False
    key = ""
    value = ""
    def __init__(self, screen: pygame.Surface, row: int, col: int, x_offset: int, width: int, height: int):
        self.screen = screen
        self.row = row
        self.col = col
        self.x_offset = x_offset
        self.width = width
        self.height = height

        self.font = pygame.font.SysFont("Arial", 20)
        self.generate_rect()


    def draw(self, get_item_clicked) -> None:
        if not self.moving:
            pygame.draw.rect(self.screen, GRID_ITEM_BACKGROUND, self.rect)
            # draw border
            pygame.draw.rect(self.screen, GRID_ITEM_BORDER, self.rect, 1)
            self.draw_grid_item_content(self.x_offset + self.col * self.width, self.row * self.height)
        else:
            # get mouse position
            (x, y) = pygame.mouse.get_pos()
            # draw center of rectangle at moues position
            self.rect = pygame.Rect(x - self.width / 2, y - self.height / 2, self.width, self.height)
            pygame.draw.rect(self.screen, GRID_ITEM_BACKGROUND, self.rect)

            # draw border
            pygame.draw.rect(self.screen, GRID_ITEM_BORDER, self.rect, 1)

            # draw grid item content
            self.draw_grid_item_content(x - self.width / 2, y - self.height / 2)

            # check if mouse is still being pressed down
            mouse_button = pygame.mouse.get_pressed()
            if not mouse_button[0]:
                self.moving = False
                # check if mouse is over another grid item
                grid_item = get_item_clicked(x, y)
                if grid_item is not None:
                    self.swap(grid_item)
                else:
                    self.generate_rect()
                
    def draw_fake_while_moving(self) -> None:
        rect = pygame.Rect(self.x_offset + self.col * self.width, self.row * self.height, self.width, self.height)
        pygame.draw.rect(self.screen, GRID_ITEM_BORDER, rect)
        # pygame.draw.rect(self.screen, (0, 0, 0), rect, 1)

    def draw_grid_item_content(self, top_left_x, top_left_y) -> None:
        # Key section
        key_rect = pygame.Rect(top_left_x, top_left_y, self.width, self.height/4)
        pygame.draw.rect(self.screen, GRID_ITEM_BORDER, key_rect)
        key_text = self.font.render(self.key, True, (255, 255, 255))
        self.screen.blit(key_text, (top_left_x + self.width/2 - key_text.get_width()/2, top_left_y + 5 + self.height/16))

        # Value section
        value_text = self.font.render(str(self.value), True, (255, 255, 255))
        self.screen.blit(value_text, (top_left_x + self.width/2 - value_text.get_width()/2, top_left_y + self.height/4 + 1.25*self.height/4))

    def swap(self, other: 'KeyValueGridItem') -> None:
        # print(f"Swap {self.key} with {other.key}")
        self.row, other.row = other.row, self.row
        self.col, other.col = other.col, self.col
        self.generate_rect()
        other.generate_rect()
        
    def generate_rect(self) -> None:
        self.x = self.x_offset + self.col * self.width
        self.y = self.row * self.height
        self.rect = pygame.Rect(self.x, self.y, self.width, self.height)

