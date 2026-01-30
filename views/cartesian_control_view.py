from PyQt5.QtWidgets import QWidget, QLabel


class CartesianControlView(QWidget):
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        self.label = QLabel("Cartesian Control View", self)
        self.label.move(50, 50)