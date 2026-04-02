from __future__ import annotations

import os

from PyQt6.QtCore import QObject
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from models.robot_model import RobotModel
from models.workspace_file import WorkspaceFile
from views.workspace_view import WorkspaceView
from widgets.workspace_view.workspace_configuration_widget import WorkspaceConfigurationWidget


class WorkspaceController(QObject):
    def __init__(self, robot_model: RobotModel, workspace_view: WorkspaceView, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.robot_model = robot_model
        self.workspace_view = workspace_view
        self.workspace_widget = workspace_view.get_configuration_widget()
        self._updating_from_view = False

        self._setup_connections()
        self._update_workspace_view()

    def _setup_connections(self) -> None:
        self.robot_model.workspace_changed.connect(self._update_workspace_view)

        self.workspace_widget.scene_name_changed.connect(self._on_scene_name_changed)
        self.workspace_widget.workspace_cad_elements_changed.connect(self._on_workspace_cad_elements_changed)
        self.workspace_widget.workspace_tcp_zones_changed.connect(self._on_workspace_tcp_zones_changed)
        self.workspace_widget.workspace_collision_zones_changed.connect(self._on_workspace_collision_zones_changed)
        self.workspace_widget.workspace_save_requested.connect(self._on_save_workspace_requested)
        self.workspace_widget.workspace_load_requested.connect(self._on_load_workspace_requested)

    def _update_workspace_view(self) -> None:
        if self._updating_from_view:
            return
        workspace_dir = self._workspace_directory()
        self.workspace_widget.set_workspace_directory(WorkspaceConfigurationWidget._normalize_project_path(workspace_dir))
        self.workspace_widget.set_workspace_scene_name(self.robot_model.get_workspace_scene_name())
        self.workspace_widget.set_workspace_file_path(self.robot_model.get_workspace_file_path())
        self.workspace_widget.set_workspace_cad_elements(self.robot_model.get_workspace_cad_elements())
        self.workspace_widget.set_workspace_tcp_zones(self.robot_model.get_workspace_tcp_zones())
        self.workspace_widget.set_workspace_collision_zones(self.robot_model.get_workspace_collision_zones())

    def _on_scene_name_changed(self, scene_name: str) -> None:
        self._updating_from_view = True
        try:
            self.robot_model.set_workspace_scene_name(scene_name)
        finally:
            self._updating_from_view = False

    def _on_workspace_cad_elements_changed(self, values: list[dict]) -> None:
        self._updating_from_view = True
        try:
            self.robot_model.set_workspace_cad_elements(values)
        finally:
            self._updating_from_view = False

    def _on_workspace_tcp_zones_changed(self, values: list[dict]) -> None:
        self._updating_from_view = True
        try:
            self.robot_model.set_workspace_tcp_zones(values)
        finally:
            self._updating_from_view = False

    def _on_workspace_collision_zones_changed(self, values: list[dict]) -> None:
        self._updating_from_view = True
        try:
            self.robot_model.set_workspace_collision_zones(values)
        finally:
            self._updating_from_view = False

    def _on_save_workspace_requested(self) -> None:
        workspace_dir = self._workspace_directory()
        scene_name = self.robot_model.get_workspace_scene_name().strip()
        if scene_name == "":
            scene_name = RobotModel.DEFAULT_WORKSPACE_SCENE_NAME
            self.robot_model.set_workspace_scene_name(scene_name)

        default_file_name = f"{self._safe_scene_file_name(scene_name)}.json"
        default_target = os.path.join(workspace_dir, default_file_name)
        selected_path, _ = QFileDialog.getSaveFileName(
            self.workspace_widget,
            "Sauvegarder un workspace",
            default_target,
            "JSON Files (*.json)",
        )
        if not selected_path:
            return
        if not selected_path.lower().endswith(".json"):
            selected_path += ".json"

        try:
            workspace_file = WorkspaceFile.from_robot_model(self.robot_model)
            workspace_file.save(selected_path)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self.workspace_widget, "Erreur sauvegarde", f"Impossible de sauvegarder la scene.\n{exc}")
            return

        self.robot_model.set_workspace_file_path(WorkspaceConfigurationWidget._normalize_project_path(selected_path))

    def _on_load_workspace_requested(self) -> None:
        workspace_dir = self._workspace_directory()
        selected_path, _ = QFileDialog.getOpenFileName(
            self.workspace_widget,
            "Charger un workspace",
            workspace_dir,
            "JSON Files (*.json)",
        )
        if not selected_path:
            return

        try:
            workspace_file = WorkspaceFile.load(selected_path)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self.workspace_widget, "Workspace invalide", f"Impossible de charger le workspace.\n{exc}")
            return

        workspace_file.apply_to_robot_model(
            self.robot_model,
            file_path=WorkspaceConfigurationWidget._normalize_project_path(selected_path),
        )

    def _workspace_directory(self) -> str:
        root_dir = os.getcwd()
        workspace_dir = os.path.abspath(os.path.join(root_dir, RobotModel.DEFAULT_WORKSPACE_DIRECTORY))
        os.makedirs(workspace_dir, exist_ok=True)
        return workspace_dir

    @staticmethod
    def _safe_scene_file_name(scene_name: str) -> str:
        forbidden = '<>:"/\\|?*'
        safe = scene_name.replace(" ", "_")
        safe = "".join("_" if c in forbidden else c for c in safe).strip().strip(".")
        return safe if safe else "scene"
