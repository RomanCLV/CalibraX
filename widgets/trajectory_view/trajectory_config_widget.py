from typing import List, Dict, Any, Optional
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QTableWidget, QTableWidgetItem, QAbstractItemView,
    QPushButton, QDoubleSpinBox, QComboBox, QSizePolicy, QDialog,
    QDialogButtonBox, QCheckBox
)
from PyQt6.QtCore import pyqtSignal


class TrajectoryKeypointDialog(QDialog):
    """Dialog to edit a single keypoint."""

    CONFIG_ORDER = ["FUN", "FUF", "FDN", "FDF", "BUN", "BUF", "BDN", "BDF"]

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        self.setWindowTitle("Keypoint")

        self.target_spins: List[QDoubleSpinBox] = []
        self.mode_combo = QComboBox()
        self.speed_spin = QDoubleSpinBox()
        self.config_checkboxes: Dict[str, QCheckBox] = {}

        self.cubic_group = QGroupBox("Vecteurs de la cubique")
        self.cubic_vector_1: List[QDoubleSpinBox] = []
        self.cubic_vector_2: List[QDoubleSpinBox] = []

        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        target_group = QGroupBox("Target (X Y Z A B C)")
        target_grid = QGridLayout()

        labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(labels):
            row = 0 if i < 3 else 1
            col = (i % 3) * 2
            target_grid.addWidget(QLabel(label), row, col)
            spin = QDoubleSpinBox()
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            self.target_spins.append(spin)
            target_grid.addWidget(spin, row, col + 1)

        target_group.setLayout(target_grid)
        layout.addWidget(target_group)

        mode_group = QGroupBox("Type de mouvement")
        mode_layout = QHBoxLayout()
        self.mode_combo.addItems(["PTP", "LINEAR", "CUBIC"])
        mode_layout.addWidget(QLabel("Mode"))
        mode_layout.addWidget(self.mode_combo)
        mode_layout.addStretch()
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        cubic_layout = QGridLayout()
        vector1Label = QLabel("Vecteur Out")
        vector1Label.setToolTip("Direction de sortie du point de départ")
        vector2Label = QLabel("Vecteur In")
        vector2Label.setToolTip("Direction d'entrée du point d'arrivée")

        cubic_layout.addWidget(vector1Label, 0, 0)
        for i in range(3):
            spin = QDoubleSpinBox()
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            self.cubic_vector_1.append(spin)
            cubic_layout.addWidget(spin, 0, i + 1)

        cubic_layout.addWidget(vector2Label, 1, 0)
        for i in range(3):
            spin = QDoubleSpinBox()
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            self.cubic_vector_2.append(spin)
            cubic_layout.addWidget(spin, 1, i + 1)

        self.cubic_group.setLayout(cubic_layout)
        layout.addWidget(self.cubic_group)

        config_group = QGroupBox("Configurations acceptées")
        config_layout = QGridLayout()
        for idx, key in enumerate(self.CONFIG_ORDER):
            row = 0 if idx < 4 else 1
            col = idx % 4
            cb = QCheckBox(key)
            cb.setChecked(True)
            self.config_checkboxes[key] = cb
            config_layout.addWidget(cb, row, col)
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)

        speed_group = QGroupBox("Vitesse")
        speed_layout = QHBoxLayout()
        self.speed_spin.setRange(0.0, 10.0)
        self.speed_spin.setDecimals(3)
        self.speed_spin.setSingleStep(0.01)
        speed_layout.addWidget(QLabel("m/s"))
        speed_layout.addWidget(self.speed_spin)
        speed_layout.addStretch()
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        layout.addWidget(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        self._update_cubic_visibility()

    def _setup_connections(self) -> None:
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)

    def _on_mode_changed(self, mode: str) -> None:
        self._update_cubic_visibility()

    def _update_cubic_visibility(self) -> None:
        self.cubic_group.setVisible(self.mode_combo.currentText() == "CUBIC")

    def _selected_configs(self) -> List[str]:
        return [key for key, cb in self.config_checkboxes.items() if cb.isChecked()]

    def get_keypoint(self) -> Dict[str, Any]:
        target = [spin.value() for spin in self.target_spins]
        vector_1 = [spin.value() for spin in self.cubic_vector_1]
        vector_2 = [spin.value() for spin in self.cubic_vector_2]
        return {
            "target": target,
            "mode": self.mode_combo.currentText(),
            "cubic_vectors": [vector_1, vector_2],
            "configs": self._selected_configs(),
            "speed": self.speed_spin.value(),
        }

    def load_keypoint(self, keypoint: Dict[str, Any]) -> None:
        target = keypoint.get("target", [0.0] * 6)
        for i, spin in enumerate(self.target_spins):
            spin.setValue(target[i] if i < len(target) else 0.0)

        self.mode_combo.setCurrentText(keypoint.get("mode", "PTP"))

        cubic_vectors = keypoint.get("cubic_vectors", [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        for i, spin in enumerate(self.cubic_vector_1):
            spin.setValue(cubic_vectors[0][i] if i < len(cubic_vectors[0]) else 0.0)
        for i, spin in enumerate(self.cubic_vector_2):
            spin.setValue(cubic_vectors[1][i] if i < len(cubic_vectors[1]) else 0.0)

        configs = keypoint.get("configs", self.CONFIG_ORDER)
        config_list = self._normalize_configs(configs)
        for key, cb in self.config_checkboxes.items():
            cb.setChecked(key in config_list)

        self.speed_spin.setValue(float(keypoint.get("speed", 0.0)))

    def _normalize_configs(self, configs: Any) -> List[str]:
        if configs is None or configs == "":
            return list(self.CONFIG_ORDER)
        if isinstance(configs, str):
            parts = [p.strip() for p in configs.replace(";", ",").split(",") if p.strip()]
            return [p for p in parts if p in self.CONFIG_ORDER]
        if isinstance(configs, list):
            return [str(p) for p in configs if str(p) in self.CONFIG_ORDER]
        return list(self.CONFIG_ORDER)


class TrajectoryConfigWidget(QWidget):
    """Widget to configure a trajectory with a list of keypoints."""

    add_requested = pyqtSignal()
    edit_requested = pyqtSignal()
    delete_requested = pyqtSignal()
    keypoints_changed = pyqtSignal(list)

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.keypoints_table = QTableWidget(0, 10)
        self.btn_add = QPushButton("Ajouter")
        self.btn_edit = QPushButton("Editer")
        self.btn_delete = QPushButton("Supprimer")

        self._keypoints: List[Dict[str, Any]] = []

        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)

        layout = QVBoxLayout(self)

        title = QLabel("Configuration de la trajectoire")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        self.keypoints_table.setHorizontalHeaderLabels([
            "#", "Mode", "Vitesse (m/s)", "X", "Y", "Z", "A", "B", "C", "Configs"
        ])
        self.keypoints_table.horizontalHeader().setDefaultSectionSize(90)
        self.keypoints_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.keypoints_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.keypoints_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.keypoints_table.setMinimumHeight(220)
        layout.addWidget(self.keypoints_table)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_add)
        btn_row.addWidget(self.btn_edit)
        btn_row.addWidget(self.btn_delete)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    def _setup_connections(self) -> None:
        self.btn_add.clicked.connect(self._on_add_clicked)
        self.btn_edit.clicked.connect(self._on_edit_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)

    def _on_add_clicked(self) -> None:
        dialog = TrajectoryKeypointDialog(self)
        dialog.load_keypoint({})
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._keypoints.append(dialog.get_keypoint())
            self._refresh_table()
            self.add_requested.emit()
            self.keypoints_changed.emit(self.get_keypoints())

    def _on_edit_clicked(self) -> None:
        row = self._selected_row()
        if row is None:
            return
        dialog = TrajectoryKeypointDialog(self)
        dialog.load_keypoint(self._keypoints[row])
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._keypoints[row] = dialog.get_keypoint()
            self._refresh_table()
            self.edit_requested.emit()
            self.keypoints_changed.emit(self.get_keypoints())

    def _on_delete_clicked(self) -> None:
        row = self._selected_row()
        if row is None:
            return
        self._keypoints.pop(row)
        self._refresh_table()
        self.delete_requested.emit()
        self.keypoints_changed.emit(self.get_keypoints())

    def _selected_row(self) -> Optional[int]:
        indexes = self.keypoints_table.selectionModel().selectedRows()
        if not indexes:
            return None
        return indexes[0].row()

    def _refresh_table(self) -> None:
        self.keypoints_table.setRowCount(0)
        for idx, kp in enumerate(self._keypoints):
            self.keypoints_table.insertRow(idx)
            values = [
                str(idx + 1),
                str(kp.get("mode", "")),
                f"{kp.get('speed', 0.0):.3f}",
            ]
            target = kp.get("target", [0.0] * 6)
            for v in target[:6]:
                values.append(f"{v:.3f}")
            configs = kp.get("configs", [])
            if isinstance(configs, list):
                config_text = ", ".join(configs)
            else:
                config_text = str(configs)
            values.append(config_text)
            for col, text in enumerate(values):
                self.keypoints_table.setItem(idx, col, QTableWidgetItem(text))

    def set_keypoints(self, keypoints: List[Dict[str, Any]]) -> None:
        self._keypoints = [dict(kp) for kp in keypoints]
        self._refresh_table()

    def get_keypoints(self) -> List[Dict[str, Any]]:
        return [dict(kp) for kp in self._keypoints]
