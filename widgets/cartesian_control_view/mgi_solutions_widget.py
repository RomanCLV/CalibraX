from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QTabWidget, QAbstractItemView, QCheckBox
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QBrush

from widgets.cartesian_control_view.mgi_configuration_selector_widget import MgiConfigurationSelectorWidget
from widgets.cartesian_control_view.mgi_joint_weights_widget import MgiJointWeightsWidget
from widgets.cartesian_control_view.mgi_jacobien_widget import MgiJacobienWidget

from utils.mgi import (
    MgiResult,
    MgiResultItem,
    MgiConfigKey,
    MgiResultStatus
)
from utils.mgi_jacobien import MgiJacobienParams, MgiJacobienResultat

# ------------------------------------------------------------
# Helpers visuels
# ------------------------------------------------------------

STATUS_COLORS = {
    MgiResultStatus.VALID: QColor("#37CA4B"),                 # vert
    MgiResultStatus.UNREACHABLE: QColor("#F13A2C"),           # rouge
    MgiResultStatus.SINGULARITY: QColor("#F13A2C"),            # orange
    MgiResultStatus.AXIS_LIMIT_VIOLATED: QColor("#F13A2C"),    # jaune
    MgiResultStatus.FORBIDDEN_CONFIGURATION: QColor("#EEAD22") # gris
}

def status_to_text(status: MgiResultStatus) -> str:
    return status.name.replace("_", " ").title()

# ------------------------------------------------------------
# Widget principal
# ------------------------------------------------------------

class MgiSolutionsWidget(QWidget):
    """
    Widget d'affichage des solutions MGI
    """

    solution_selected = pyqtSignal(MgiConfigKey)
    solution_item_selected = pyqtSignal(MgiConfigKey, list)
    allowed_configs_changed = pyqtSignal()
    weights_changed = pyqtSignal(list)
    jacobien_enabled_changed = pyqtSignal(bool)
    jacobien_params_changed = pyqtSignal()

    def __init__(self, parent: QWidget=None):
        super().__init__(parent)

        self._mgi_result: MgiResult | None = None
        self._selected_key: MgiConfigKey | None = None
        self._selected_joints: list[float] | None = None
        self._axis_limits = [(-180., 180.) for _ in range(6)]
        self._show_expanded_solutions = False

        self._tabs = QTabWidget()
        self._solutions_tab = QWidget()
        self._filters_tab = QWidget()
        self._weights_tab = QWidget()

        self._allowed_configs = {key: True for key in MgiConfigKey}
        self._joint_weights = [1.0] * 6

        self._config_selector = MgiConfigurationSelectorWidget(self._allowed_configs)
        self._weights_selector = MgiJointWeightsWidget(self._joint_weights)

        self._jacobien_tab = QWidget()

        self._init_solutions_tab()
        self._init_filters_tab()
        self._init_weights_tab()
        self._init_jacobien_tab()

        self._tabs.addTab(self._solutions_tab, "Solutions")
        self._tabs.addTab(self._filters_tab, "Configurations acceptées")
        self._tabs.addTab(self._weights_tab, "Poids des joints")
        self._tabs.addTab(self._jacobien_tab, "MGI Optimisé")

        layout = QVBoxLayout(self)
        layout.addWidget(self._tabs)

    # --------------------------------------------------------
    # Public API
    # --------------------------------------------------------

    def get_config_selector(self) -> MgiConfigurationSelectorWidget:
        return self._config_selector

    def set_axis_limits(self, limits: list[tuple[float, float]]):
        self._axis_limits = limits

    def set_mgi_result(self, result: MgiResult, selected_key: MgiConfigKey|None, selected_joints: list[float] | None = None):
        self._mgi_result = result
        self._selected_key = selected_key
        if selected_joints is None:
            self._selected_joints = None
        else:
            joints = [float(v) for v in selected_joints[:6]]
            while len(joints) < 6:
                joints.append(0.0)
            self._selected_joints = joints
        self._populate_table()
    
    def set_selected_key(self, selected_key: MgiConfigKey|None, selected_joints: list[float] | None = None):
        self._selected_key = selected_key
        if selected_joints is not None:
            joints = [float(v) for v in selected_joints[:6]]
            while len(joints) < 6:
                joints.append(0.0)
            self._selected_joints = joints
        if self._mgi_result:
            self._populate_table()
    
    def set_weights(self, weights: list[float]):
        self._joint_weights = list(weights)
        self._weights_selector.set_weights(weights)

    def get_allowed_configs(self):
        return self._allowed_configs

    # --------------------------------------------------------
    # Solutions tab
    # --------------------------------------------------------

    def _init_solutions_tab(self):
        layout = QVBoxLayout(self._solutions_tab)

        self._cb_show_expanded = QCheckBox("Afficher solutions étendues")
        self._cb_show_expanded.setChecked(False)
        self._cb_show_expanded.setToolTip(
            "Affiche les variantes équivalentes (tours +/-360) quand disponibles."
        )
        self._cb_show_expanded.toggled.connect(self._on_show_expanded_toggled)
        layout.addWidget(self._cb_show_expanded)

        self._table = QTableWidget()
        self._table.setColumnCount(9)
        self._table.horizontalHeader().setDefaultSectionSize(110)
        self._table.setHorizontalHeaderLabels([
            "Config",
            "Statut",
            "q1", "q2", "q3", "q4", "q5", "q6",
            "Action"
        ])

        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)

        layout.addWidget(self._table)

    def _populate_table(self):
        self._table.setRowCount(0)

        if self._mgi_result is None:
            return
        
        if self._show_expanded_solutions and self._mgi_result.expanded_solutions:
            config_order = {key: idx for idx, key in enumerate(MgiConfigKey)}
            rows: list[tuple[MgiConfigKey, MgiResultItem]] = []
            for item in self._mgi_result.expanded_solutions:
                if item.config_key is None:
                    continue
                rows.append((item.config_key, item))
            rows.sort(key=lambda pair: (config_order.get(pair[0], 999), pair[1].joints))
        else:
            rows = list(self._mgi_result.solutions.items())

        selected_row_index = self._resolve_selected_row_index(rows)

        for row, (config_key, item) in enumerate(rows):
            self._table.insertRow(row)

            col_index = 0

            # --- Configuration
            configuration_item = QTableWidgetItem(config_key.name)
            is_selected_row = selected_row_index is not None and row == selected_row_index
            configuration_item.setForeground(QBrush(QColor("orange" if is_selected_row else "white")))
            self._table.setItem(row, col_index, configuration_item)
            
            col_index += 1

            # --- Statut
            status_item = QTableWidgetItem(status_to_text(item.status))
            status_item.setForeground(QBrush(STATUS_COLORS.get(item.status, QColor("white"))))
            status_item.setToolTip(self._build_status_tooltip(item))
            self._table.setItem(row, col_index, status_item)
            col_index += 1

            # --- Joints
            for joint in item.joints:
                joint_item = QTableWidgetItem(f"{joint:.3f}")
                joint_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self._table.setItem(row, col_index, joint_item)
                col_index += 1

            # --- Bouton sélectionner
            btn = QPushButton("Sélectionner")
            btn.setEnabled(item.status == MgiResultStatus.VALID)
            if not btn.isEnabled():
                btn.setStyleSheet("color: gray")
            
            row_joints = [float(v) for v in item.joints[:6]]
            while len(row_joints) < 6:
                row_joints.append(0.0)
            btn.clicked.connect(
                lambda _, k=config_key, j=row_joints: self._emit_solution_selection(k, j)
            )
            self._table.setCellWidget(row, col_index, btn)
            col_index += 1

        self._table.resizeColumnsToContents()

    def _on_show_expanded_toggled(self, checked: bool) -> None:
        self._show_expanded_solutions = bool(checked)
        self._populate_table()

    def _emit_solution_selection(self, config_key: MgiConfigKey, joints: list[float]) -> None:
        copied = [float(v) for v in joints[:6]]
        while len(copied) < 6:
            copied.append(0.0)
        self.solution_item_selected.emit(config_key, copied)
        # Compat: conserve l'ancien signal base sur la config uniquement.
        self.solution_selected.emit(config_key)

    def _resolve_selected_row_index(self, rows: list[tuple[MgiConfigKey, MgiResultItem]]) -> int | None:
        if not rows:
            return None

        # Priorite: retrouver la solution qui correspond vraiment aux angles courants.
        # Cela reste fiable meme si la cle de config est stale ou ambigue.
        if self._selected_joints is not None:
            selected = [float(v) for v in self._selected_joints[:6]]
            while len(selected) < 6:
                selected.append(0.0)

            weights = [float(v) for v in self._joint_weights[:6]]
            while len(weights) < 6:
                weights.append(1.0)

            exact_candidates: list[int] = []
            best_idx = 0
            best_distance = float("inf")

            for idx, (_, item) in enumerate(rows):
                joints = [float(v) for v in item.joints[:6]]
                while len(joints) < 6:
                    joints.append(0.0)

                deltas = [selected[axis] - joints[axis] for axis in range(6)]
                max_abs_delta = max(abs(v) for v in deltas)
                if max_abs_delta <= 1e-6:
                    exact_candidates.append(idx)

                distance = 0.0
                for axis in range(6):
                    distance += weights[axis] * (deltas[axis] ** 2)

                if distance < best_distance:
                    best_distance = distance
                    best_idx = idx

            if exact_candidates:
                if self._selected_key is not None:
                    for idx in exact_candidates:
                        cfg, _ = rows[idx]
                        if cfg == self._selected_key:
                            return idx
                return exact_candidates[0]

            return best_idx

        if self._selected_key is None:
            return None

        for idx, (cfg, _) in enumerate(rows):
            if cfg == self._selected_key:
                return idx

        return None

    def _build_status_tooltip(self, item: MgiResultItem) -> str:
        lines = [f"Statut : {item.status.name}"]

        if item.j1Singularity:
            lines.append("• Singularité Q1")
        if item.j3Singularity:
            lines.append("• Singularité Q3")
        if item.j5Singularity:
            lines.append("• Singularité Q5")

        if item.violated_limits and len(item.violated_limits) > 0:
            lines.append("Axes hors limites :")
            for violated_axis in item.violated_limits:
                min, max = self._axis_limits[violated_axis]
                lines.append(f"• J{violated_axis+1}: {item.joints[violated_axis]} ({min}, {max})")

        return "\n".join(lines)

    # --------------------------------------------------------
    # Filters tab (base)
    # --------------------------------------------------------

    def _init_filters_tab(self):
        layout = QVBoxLayout(self._filters_tab)

        self._config_selector = MgiConfigurationSelectorWidget(
            {key: True for key in MgiConfigKey}
        )

        self._config_selector.configurations_changed.connect(self._on_configurations_changed)

        layout.addWidget(self._config_selector)
        layout.addStretch()

    def _init_weights_tab(self):
        layout = QVBoxLayout(self._weights_tab)

        self._weights_selector = MgiJointWeightsWidget(self._joint_weights)

        self._weights_selector.weights_changed.connect(
            self._on_weights_changed
        )

        layout.addWidget(self._weights_selector)
        layout.addStretch()

    def _on_configurations_changed(self, states: dict):
        """
        Callback lorsque l'utilisateur modifie les configurations autorisées
        """
        self._allowed_configs = {k for k, v in states.items() if v}
        self.allowed_configs_changed.emit()

    def _on_weights_changed(self, weights: list):
        """
        Callback lorsque l'utilisateur modifie les poids des joints
        """
        self._joint_weights = weights
        self.weights_changed.emit(weights)

    # --------------------------------------------------------
    # Onglet MGI Optimisé (Jacobienne)
    # --------------------------------------------------------

    def _init_jacobien_tab(self):
        layout = QVBoxLayout(self._jacobien_tab)

        self._jacobien_widget = MgiJacobienWidget()
        self._jacobien_widget.enabled_changed.connect(self.jacobien_enabled_changed)
        self._jacobien_widget.params_changed.connect(self.jacobien_params_changed)

        layout.addWidget(self._jacobien_widget)

    def get_jacobien_widget(self) -> MgiJacobienWidget:
        """Retourne le widget de configuration du solveur MGI Jacobienne."""
        return self._jacobien_widget

    def is_jacobien_enabled(self) -> bool:
        """Retourne True si le solveur MGI Jacobienne est activé."""
        return self._jacobien_widget.is_enabled()

    def get_jacobien_params(self) -> MgiJacobienParams:
        """Retourne les paramètres actuels du solveur MGI Jacobienne."""
        return self._jacobien_widget.get_params()

    def set_jacobien_resultat(self, resultat: MgiJacobienResultat | None):
        """Met à jour l'affichage de convergence du solveur MGI Jacobienne."""
        self._jacobien_widget.set_resultat(resultat)
