from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QTableWidget,
    QTableWidgetItem, QAbstractItemView
)
from PyQt6.QtCore import pyqtSignal


class CorrectionTableWidget(QWidget):
    """Widget pour afficher et éditer les corrections 6D"""

    # Émis quand l'utilisateur modifie une cellule.
    # Transmet la table complète sous forme de floats : list[list[float]]
    corrections_changed = pyqtSignal(list)

    def __init__(self, parent: QWidget=None):
        super().__init__(parent)
        self._updating_from_model = False  # Garde contre la boucle modèle→vue→modèle
        self.setup_ui()

    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)

        # Titre
        titre5 = QLabel("Corrections 6D")
        titre5.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre5)

        # Table des corrections
        self.table_corr = QTableWidget(6, 6)
        self.table_corr.setEditTriggers(QAbstractItemView.EditTrigger.AllEditTriggers)
        self.table_corr.setHorizontalHeaderLabels(["Tx(mm)", "Ty(mm)", "Tz(mm)", "Rx(°)", "Ry(°)", "Rz(°)"])
        self.table_corr.horizontalHeader().setDefaultSectionSize(80)
        self.table_corr.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_corr.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_corr.itemChanged.connect(self._on_item_changed)

        layout.addWidget(self.table_corr)
        self.setLayout(layout)

    def set_corrections(self, corrections: list[list]):
        """Charge les corrections dans la table (depuis le modèle — ne réémet pas le signal)."""
        self._updating_from_model = True
        try:
            for i in range(min(6, len(corrections))):
                for j in range(6):
                    value = str(corrections[i][j]) if j < len(corrections[i]) else "0"
                    item = self.table_corr.item(i, j)
                    if item is None:
                        self.table_corr.setItem(i, j, QTableWidgetItem(value))
                    else:
                        item.setText(value)
        finally:
            self._updating_from_model = False

    def get_corrections(self) -> list[list[float]]:
        """Récupère les corrections depuis la table sous forme de floats."""
        corrections: list[list[float]] = []
        for i in range(6):
            row: list[float] = []
            for j in range(6):
                item = self.table_corr.item(i, j)
                try:
                    row.append(float(item.text()) if item and item.text().strip() != "" else 0.0)
                except ValueError:
                    row.append(0.0)
            corrections.append(row)
        return corrections

    def _on_item_changed(self, *_):
        """Propagation vers le modèle quand l'utilisateur édite une cellule."""
        if self._updating_from_model:
            return
        self.corrections_changed.emit(self.get_corrections())
