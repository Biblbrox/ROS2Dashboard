from abc import ABC, abstractmethod

class Visualizer(ABC):
    def __init__(self) -> None:
        super().__init__()
        self.widget = None
        self.is_active = False

    @abstractmethod
    def update_widget(self, data) -> None:
        pass

