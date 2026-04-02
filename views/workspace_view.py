from PyQt6.QtWidgets import QScrollArea, QVBoxLayout, QWidget

from widgets.workspace_view.workspace_configuration_widget import WorkspaceConfigurationWidget


class WorkspaceView(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.configuration_widget = WorkspaceConfigurationWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self.configuration_widget)
        layout.addWidget(scroll_area)

    def get_configuration_widget(self) -> WorkspaceConfigurationWidget:
        return self.configuration_widget
